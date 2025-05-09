/*
 * gnss.c
 *
 *  Created on: Mar 27, 2025
 *      Author: theo8
 */

#include <gnss.h>

/* Explications :
Le module GNSS récupère des données (GeoPoint). Ces points forment une track, cette track va être filtrée.
Pour filtrer cette track, il nous faudra :

1- Passer chaque point en coordonnées UTM initialement en coordonnées géographiques (LAT/LONG).

2- Filtrer les points abérrants (outliners). Calculer la distance entre deux points qui se suivent (selon le temps), si cette distance est trop élevée
notre module GNSS a surement fait une erreur de récupération de coordonnées.
Ex : 5 mètres entre deux points sur une durée Delta_t=0.25s cela signifirait une vitesse de 20m.s (~70 km/h), ce n'est pas cohérent, on enlève le point.

3- Appliquer le filtrage Douglas-Peucker qui consiste à enlever les points qui n'apportent aucune information supplémentaire.
Ex : 5 points, A B C D E -> On prend comme départ/fin A/E, on trace une droite entre A & E.
B & D sont superposés à la droite, alors que C s'en écarte un peu. Pour diminuer le nombre de points de la track tout en gardant la direction générale.
On ne va garder que A C E.

4- Repasser en coordonnées géographiques. On obtient de cette façon des points qui forment une track que l'on peut traduire en ordre moteur pour avoir
une track autonome.


*/

// Constantes pour la conversion UTM
#define WGS84_A 6378137.0           // Demi-grand axe (m)
#define WGS84_E 0.081819191         // Excentricité
#define UTM_SCALE_FACTOR 0.9996     // Facteur d'échelle UTM
#define UTM_ZONE_31N_MERIDIAN 3.0   // Méridien central (degrés)

// Buffer pour stocker les points GNSS
#define MAX_POINTS 100
GeoPoint gnssTrack[MAX_POINTS];
UTMPoint utmTrack[MAX_POINTS];
int pointCount = 0;

// ==================== CONVERSION WGS84 → UTM ========================================================================================
UTMPoint geoToUTM(GeoPoint geo) {
    UTMPoint utm;
    double latRad = geo.latitude * M_PI / 180.0;
    double lonRad = geo.longitude * M_PI / 180.0;

    // Calcul du fuseau UTM (1-60)
    utm.zone = (int)(geo.longitude + 180.0) / 6 + 1;

    // Bande latitudinale (C à X)
    if (geo.latitude >= -80 && geo.latitude < 84) {
        utm.band = 'C' + (int)((geo.latitude + 80) / 8);
    } else {
        utm.band = 'Z'; // Cas spécial (pôles)
    }

    // Conversion UTM (simplifiée, utilisez une librairie précise en production)
    double N = WGS84_A / sqrt(1 - WGS84_E * WGS84_E * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = WGS84_E * WGS84_E * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - ((utm.zone - 1) * 6 - 3 + 180) * M_PI / 180.0);

    double M = WGS84_A * ((1 - WGS84_E * WGS84_E / 4 - 3 * pow(WGS84_E, 4) / 64 - 5 * pow(WGS84_E, 6) / 256) * latRad
                  - (3 * WGS84_E * WGS84_E / 8 + 3 * pow(WGS84_E, 4) / 32 + 45 * pow(WGS84_E, 6) / 1024) * sin(2 * latRad)
                  + (15 * pow(WGS84_E, 4) / 256 + 45 * pow(WGS84_E, 6) / 1024) * sin(4 * latRad)
                  - (35 * pow(WGS84_E, 6) / 3072) * sin(6 * latRad));

    utm.easting = UTM_SCALE_FACTOR * N * (A + (1 - T + C) * pow(A, 3) / 6
                  + (5 - 18 * T + T * T + 72 * C - 58 * WGS84_E * WGS84_E) * pow(A, 5) / 120)
                  + 500000.0;

    utm.northing = UTM_SCALE_FACTOR * (M + N * tan(latRad) * (A * A / 2
                     + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24
                     + (61 - 58 * T + T * T + 600 * C - 330 * WGS84_E * WGS84_E) * pow(A, 6) / 720));

    if (geo.latitude < 0) {
        utm.northing += 10000000.0; // Correction hémisphère Sud
    }

    return utm;
}

// ==================== FILTRAGE DES OUTLIERS ===============================================================================================
// Distance entre deux points UTM (en mètres)
double distanceUTM(UTMPoint a, UTMPoint b) {
    return sqrt(pow(a.easting - b.easting, 2) + pow(a.northing - b.northing, 2));
}

// Supprime les points trop éloignés (seuil en mètres)
void removeOutliers(UTMPoint *points, int *count, double threshold) {
    if (*count < 3) return;

    for (int i = 1; i < *count - 1; i++) {
        double prevDist = distanceUTM(points[i-1], points[i]);
        double nextDist = distanceUTM(points[i], points[i+1]);

        if (prevDist > threshold || nextDist > threshold) {
            // Décalage pour supprimer le point aberrant
            for (int j = i; j < *count - 1; j++) {
                points[j] = points[j+1];
            }
            (*count)--;
            i--;
        }
    }
}

// ==================== SIMPLIFICATION (DOUGLAS-PEUCKER) =========================================================================================
// Distance d'un point perpendiculaire à une droite formée de deux points(en mètres)
double perpendicularDistance(UTMPoint p, UTMPoint a, UTMPoint b) {
    // Vecteur AB
    double abx = b.easting - a.easting;
    double aby = b.northing - a.northing;

    // Vecteur AP
    double apx = p.easting - a.easting;
    double apy = p.northing - a.northing;

    // Produit scalaire pour la projection
    double dot = apx * abx + apy * aby;
    double abLenSq = abx * abx + aby * aby;
    double t = fmax(0.0, fmin(1.0, dot / abLenSq));

    // Point projeté
    double px = a.easting + t * abx;
    double py = a.northing + t * aby;

    // Distance euclidienne
    return sqrt((p.easting - px) * (p.easting - px) +
                (p.northing - py) * (p.northing - py));
}

// Algorithme récursif pour simplifier une trace
void douglasPeucker(UTMPoint *points, int start, int end, double epsilon, bool *keep) {
    if (start >= end - 1) return;

    double dmax = 0.0;
    int index = start;

    for (int i = start + 1; i < end; i++) {
        double d = perpendicularDistance(points[i], points[start], points[end]);
        if (d > dmax) {
            dmax = d;
            index = i;
        }
    }

    if (dmax > epsilon) {
        keep[index] = true;
        douglasPeucker(points, start, index, epsilon, keep);
        douglasPeucker(points, index, end, epsilon, keep);
    }
}

// Applique la simplification
void simplifyTrack(UTMPoint *points, int *count, double epsilon) {
    if (*count < 3) return;

    bool *keep = (bool *)calloc(*count, sizeof(bool));
    keep[0] = keep[*count - 1] = true; // Garde les extrémités

    douglasPeucker(points, 0, *count - 1, epsilon, keep);

    // Réécrit les points conservés
    int newCount = 0;
    for (int i = 0; i < *count; i++) {
        if (keep[i]) {
            points[newCount++] = points[i];
        }
    }
    *count = newCount;
    free(keep);
}

// ==================== CONVERSION UTM → WGS84 ===============================================================================================
GeoPoint utmToGeo(UTMPoint utm) {
    GeoPoint geo;
    double x = utm.easting - 500000.0;  // Retrait du faux Esting
    double y = utm.northing;

    // Calcul de la latitude (φ) via itération
    double phi = y / (WGS84_A * UTM_SCALE_FACTOR);
    double phiPrev = 0.0;
    int maxIter = 10;
    double tolerance = 1e-8;

    for (int i = 0; i < maxIter; i++) {
        double M = WGS84_A * (
            (1 - WGS84_E*WGS84_E/4 - 3*pow(WGS84_E,4)/64 - 5*pow(WGS84_E,6)/256) * phi
            - (3*WGS84_E*WGS84_E/8 + 3*pow(WGS84_E,4)/32 + 45*pow(WGS84_E,6)/1024) * sin(2*phi)
            + (15*pow(WGS84_E,4)/256 + 45*pow(WGS84_E,6)/1024) * sin(4*phi)
            - (35*pow(WGS84_E,6)/3072) * sin(6*phi)
        );

        phiPrev = phi;
        phi = (y - M) / (WGS84_A * UTM_SCALE_FACTOR) + phi;

        if (fabs(phi - phiPrev) < tolerance) break;
    }

    // Calcul de la longitude (λ)
    double N = WGS84_A / sqrt(1 - WGS84_E*WGS84_E * sin(phi)*sin(phi));
    double T = tan(phi)*tan(phi);
    double C = WGS84_E*WGS84_E * cos(phi)*cos(phi);
    double A = x / (N * UTM_SCALE_FACTOR);
    double lonRad = A - (1 - T + C) * pow(A,3)/6
                    + (5 - 18*T + T*T + 72*C - 58*WGS84_E*WGS84_E) * pow(A,5)/120;

    // Conversion en degrés
    geo.latitude = phi * 180.0 / M_PI;
    geo.longitude = UTM_ZONE_31N_MERIDIAN + lonRad * 180.0 / M_PI;

    return geo;
}

// ==================== BOUCLE PRINCIPALE ===============================================================================================

void processGNSS(GeoPoint *output, int *numPoints) {
    // 1. Simulation acquisition GNSS
    pointCount = 10;
    for (int i = 0; i < pointCount; i++) {
        gnssTrack[i].latitude = 48.8584 + (rand() % 100) * 0.0001;
        gnssTrack[i].longitude = 2.2945 + (rand() % 100) * 0.0001;
    }

    // 2. Conversion en UTM
    for (int i = 0; i < pointCount; i++) {
        utmTrack[i] = geoToUTM(gnssTrack[i]);
    }

    // 3. Filtrage des outliers (100m)
    removeOutliers(utmTrack, &pointCount, 100.0);

    // 4. Simplification (10m)
    simplifyTrack(utmTrack, &pointCount, 10.0);

    // 5. Conversion finale et stockage
    for (int i = 0; i < pointCount; i++) {
        output[i] = utmToGeo(utmTrack[i]);
    }
    *numPoints = pointCount;
}
