# ParisBioTek — Pilote de voltampérométrie cyclique (CV)

Logiciel PC pour piloter la carte **nRF52840 + AD5940** par liaison série (USB).
On règle les paramètres de la CV, on lance la mesure, on voit la courbe
courant-tension **en direct**, et on enregistre en `.csv`.

## Installation (une seule fois)

1. Installer Python 3.9+ (cocher « Add Python to PATH » à l'installation).
2. Dans un terminal, depuis ce dossier :

   ```
   pip install -r requirements.txt
   ```

## Lancer

```
python main.py
```

## Utilisation

1. Brancher la carte en USB.
2. **Détecter** (ou choisir le port COM puis **Connecter**). La carte qui répond
   au test d'identification est reconnue automatiquement.
3. Régler les paramètres de balayage :
   - *Tension départ / sommet (mV)* : bornes du triangle de potentiel.
   - *Vzero / biais (mV)* : polarisation de référence.
   - *Nombre de pas* (pair).
   - *Vitesse (mV/s)* : la durée du cycle est calculée et affichée.
   - *Délai d'échantillon (ms)*.
4. **Lancer la mesure** : les points s'affichent au fur et à mesure (~la durée du
   cycle). À la fin, le pic de courant et la résistance estimée s'affichent.
5. Le `.csv` est enregistré automatiquement dans `mesures/` (désactivable), ou
   via **Enregistrer CSV…**.

## Format du CSV

```
# start_mv=... peak_mv=... vzero_mv=... steps=... duration_ms=... sample_delay_ms=...
# CV,RESULT,peak_nA=...,R_ohm=...
index,voltage_mV,current_nA,current_uA
0,-1000.0,12.3,0.0123
...
```

## Notes

- Le même port USB sert aussi au **DFU mcumgr** (mise à jour du firmware). Fermez
  ce logiciel (ou cliquez **Déconnecter**) avant de flasher la carte, pour libérer
  le port.
- Les gains matériels (RTIA, PGA, RCAL, Vref) restent calibrés dans le firmware et
  ne sont pas réglables ici (volontairement).
- Protocole série complet documenté en tête de `main.py`.
