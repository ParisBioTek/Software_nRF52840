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
   - *Nombre de pas* (pair) : nombre de points d'un cycle complet.
   - *Cycles* : nombre de triangles départ → sommet → départ à enchaîner.
   - *Vitesse (mV/s)* : la durée **par cycle** est calculée et affichée.
   - *Délai d'échantillon (ms)*.
   - *Stabilisation initiale (ms)* : maintien au potentiel de départ avant le
     premier échantillon (`2000` par défaut, `0` pour désactiver).
   Le produit `nombre de pas × cycles` ne doit pas dépasser **4000 points**. La durée
   totale vaut approximativement `stabilisation + durée par cycle × cycles`.
4. **Lancer la mesure** : les points s'affichent au fur et à mesure (~la durée
   totale). À la fin, le pic de courant et la résistance estimée s'affichent.
5. **Arrêter la mesure** envoie un STOP coopératif au firmware. Tous les points
   confirmés sont récupérés et restent enregistrables ; ce n'est pas une erreur.
6. Le `.csv` est enregistré automatiquement dans `mesures/` (désactivable), ou
   via **Enregistrer CSV…**.

La tendance moyenne CV est activée par défaut dans le cadre **Mesure**. Elle
combine une médiane robuste sur environ 21 mV, qui retire les pics et les petites
salves, avec une moyenne gaussienne centrée sur environ 45 mV. Chaque montée,
descente et cycle est traité séparément. La case peut être décochée pour retrouver
instantanément le tracé brut.

Le titre du graphique est en anglais (`Cyclic voltammetry`, `Chronoamperometry`,
`Differential pulse voltammetry`) ; les axes et le reste de l'interface restent
en français.

## Journal et diagnostic des erreurs

Le panneau **Journal** affiche désormais la phase firmware, la cadence réelle, le
`run_id`, le détail AD5940, la LFOSC mesurée/utilisée, les erreurs SPI et les drapeaux
séquenceur/FIFO. Il peut être copié ou enregistré depuis l'interface. Un journal quotidien
persistant est aussi écrit dans `journaux/`.

Depuis le firmware `20260824-clk-v5` (`diag_ver=5`), le journal indique aussi ADIID,
CHIPID, les quatre emplacements d'identifiant SPI, le masque exact des modes
réellement essayés (`mode_tried`), le mode retenu, le retour des commandes GPIO,
les latches PWR/CS/RESET et l'état de commande OE d'OSC1 (`hfxo_on`). `gpio_state`
et `hfxo_on` sont des états logiciels des sorties nRF, pas des mesures de tensions.
Si une mesure s'arrête à
`*/platform_init` avec tous les identifiants à `0000` ou `FFFF`, le message sépare
explicitement cette panne capteur/alimentation d'une panne USB, SMP, Python ou d'un
mauvais réglage CV.

L'image `20260824-pwr-v4` est obsolète : elle coupe P0.08, OE de l'oscillateur
32 MHz du nRF, à la fin de chaque acquisition et fige le CPU/USB avant le STATUS
final. La GUI la détecte à la connexion et interdit désormais START jusqu'au flash
de `20260825-directinit-v8`. Les erreurs série indiquent la commande SMP, la séquence, la phase
lecture/écriture, l'état du port Windows, les octets tentés/réellement écrits et les
compteurs timeout/CRC/bruit/protocole.

Une perte ponctuelle de réponse `STATUS` est relancée jusqu'à trois fois. Une
trame série abîmée au moment où la carte passe de `run=true` à `complete` ne
transforme donc plus une acquisition complète en rapport `_ERREUR`.

Après un échec, le logiciel conserve et trace les points déjà reçus puis crée
automatiquement, sous `mesures/<session>/<test>/`, un rapport `.txt`, le contexte complet
`_diagnostic.json` et, s'il existe des points, un `_partiel.csv`.

Avec `20260825-directinit-v8` (`diag_ver=6`), START et STATUS confirment aussi
`settle_ms`; STATUS expose `stop_requested` et `cancelled`. Un arrêt par le bouton
est autosauvegardé avec le suffixe `_INTERROMPUE` et ne crée pas de rapport
`_ERREUR`.

Le même build rend l'initialisation du moteur CV tolérante à un `ENDSEQ` non
observé : `SEQ_STOP`/`SEQEN=0` reste une preuve matérielle de fin. Si le
déclenchement MMR de SEQ3 ne démarre pas, il applique l'initialisation par SPI
et conserve les séquences ADC/DAC WUPT. Le journal ajoute `init_wait_polls`,
`init_endseq_missing`, `init_direct_fallback` et `SEQ3INFO` pour documenter
cette décision.

### Câblage d'une résistance étalon

Pour vérifier la chaîne de mesure sans cellule électrochimique :

- court-circuiter **CE0 et RE0** ;
- placer la résistance entre ce nœud **CE0–RE0** et **SE0** ;
- laisser **DE0 ouvert** (non connecté).

Ne pas relier RE0 à SE0. Avec le câblage correct, la CV d'une résistance doit être une
droite `I = V/R`, et non deux plateaux de courant.

## Format du CSV

```
# start_mv=... peak_mv=... vzero_mv=... steps=... cycles=... duration_ms=... sample_delay_ms=... settle_ms=...
# CV,RESULT,peak_nA=...,R_ohm=...
index,voltage_mV,current_nA,current_uA,current_filtered_nA,current_filtered_uA
0,-1000.0,12.3,0.0123,12.1,0.0121
...
```

`steps` et `duration_ms` décrivent **un cycle**. Avec `cycles=1`, le format ci-dessus est
conservé. Avec plusieurs cycles, la colonne `cycle`, numérotée à partir de 1, est ajoutée :

```
index,cycle,voltage_mV,current_nA,current_uA,current_filtered_nA,current_filtered_uA
0,1,-1000.0,12.3,0.0123,12.1,0.0121
...
100,2,-1000.0,12.1,0.0121,12.0,0.0120
```

`current_nA` et `current_uA` sont toujours les mesures brutes. Les colonnes
`current_filtered_*` contiennent la tendance moyenne : médiane robuste sur
environ 21 mV, puis moyenne gaussienne centrée sur environ 45 mV. Le traitement
est effectué séparément sur chaque branche et chaque cycle.

## Notes

- Le même port USB sert aussi au **DFU mcumgr** (mise à jour du firmware). Fermez
  ce logiciel (ou cliquez **Déconnecter**) avant de flasher la carte, pour libérer
  le port.
- La gamme **RTIA** est réglable dans l'interface. Le PGA, la RCAL et la référence
  ADC restent des paramètres de calibration du firmware.
- Protocole série complet documenté en tête de `main.py`.
