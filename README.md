French version Below

# ParisBioTek — nRF52840 + AD5940 firmware & voltammetry PC tool
 
**Electrochemical measurement** platform on the nRF52840 (Nordic), driven from a PC:
a microcontroller runs the techniques (cyclic voltammetry, chronoamperometry,
DPV) on an **AD5940** potentiostat, and a Python tool triggers / retrieves / plots
them over USB — **on the same channel as the firmware update (DFU)**.
 
> This repo covers the **firmware** and the **PC tool**. The companion Android app
> (NFC + BLE authentication) is not needed to develop here.
> PCBs deigned with EasyEDA Pro available here :
 
---
 
## 1. Overview
 
```mermaid
flowchart LR
    subgraph PC
      A[CV_python/main.py<br/>Tkinter + matplotlib]
    end
    subgraph nRF52840
      B[SMP group « CV » id 64] --> C[Dispatcher run_measurement]
      C --> D[CV.c / CA.c / DPV.c]
      D --> E[ad5940_port / ad5940_lp<br/>SPI]
    end
    subgraph Sensor
      F[AD5940 potentiostat] --> G[Electrodes CE0 / RE0 / SE0]
    end
    A <-->|USB CDC : SMP framing<br/>base64 + CRC16| B
    E <-->|SPI mode 0| F
```
 
- A **single USB port (CDC ACM)** serves both the **mcumgr DFU** and **measurement
  control** (custom SMP group `id 64`). Only one program can open the port at a time.
- **Logs / console** go out over **RTT (J-Link)**, never over USB (otherwise the SMP
  frames would be corrupted).
---
 
## 2. Directory layout
 
```
firmware/                     ← this folder (NCS/Zephyr project, sysbuild + MCUboot)
├── CMakeLists.txt
├── prj.conf                  ← Bluetooth/NFC/USB/mcumgr config + RTT
├── app.overlay               ← 1 CDC ACM (console+DFU+CV), SPI1 AD5940, GPIO
├── flash.ps1                 ← USB DFU flash in 1 command (upload→test→reset)
├── mcumgr-client.exe         ← serial DFU client (0.0.9)
└── src/
    ├── main.c                ← USB/BLE/NFC + SMP CV group + dispatcher
    ├── methods.h             ← technique enum (CV=0, CA=1, DPV=2)
    ├── CV.c / CV.h           ← cyclic voltammetry (AD5940 sequencer)
    ├── CA.c / CA.h           ← chronoamperometry (software-timed)
    ├── DPV.c / DPV.h         ← differential pulse voltammetry (software-timed)
    ├── ad5940_lp.c/.h        ← low-power loop primitives (LPDAC+LPTIA) + RTIA
    ├── RampTest.c/.h         ← ADI « ramp » engine (used by CV)
    ├── ad5940.c/.h           ← ADI AD5940 driver
    └── ad5940_port.c/.h      ← Zephyr SPI port + sensor power
 
CV_python/                    ← PC tool (sibling folder)
└── main.py                   ← control / plot / export GUI
```
 
---
 
## 3. Hardware
 
| Item          | Detail                                                              |
|---------------|--------------------------------------------------------------------|
| Board         | **nRF52840 DK** (`nrf52840dk/nrf52840`)                             |
| Sensor        | **AD5940** (potentiostat) on **SPI1**, mode 0                       |
| Programming   | **J-Link** (VCOM/RTT) *or* **USB DFU** (mcumgr over the native CDC) |
| Electrodes    | CE0 (counter electrode), RE0 (reference), SE0 (working)             |
 
SPI wiring (see `app.overlay`): `SCLK=P0.13  MOSI=P0.22  MISO=P0.15  CS=P0.17`,
reset `P0.20`, sensor power `P1.00`.
 
**"Resistor" test** (bring-up without a real cell): place a reference resistor
between **CE0** and the node **(RE0 tied to SE0)**. A CV should then give a straight
line whose slope = the resistance. The RTIA is calibrated via an **external RCAL**
(`CV_RCAL_OHM` in `src/CV.h`) — set this value to the RCAL actually wired for accuracy.
 
---
 
## 4. Software prerequisites
 
### Firmware — nRF Connect SDK **v3.3.0**
Install via Nordic's **Toolchain Manager** (or the *nRF Connect for VS Code* extension):
- SDK: `C:\ncs\v3.3.0`
- Toolchain: `C:\ncs\toolchains\936afb6332`
`west` / `nrfutil` are not on the global PATH: open an SDK shell with
```powershell
nrfutil toolchain-manager launch --shell
```
or simply **open the `firmware/` folder in VS Code + nRF Connect** (build/flash in one click).
 
### PC tool — Python 3.9+
```bash
pip install pyserial cbor2 matplotlib
```
(`tkinter` ships with Python on Windows.)
 
---
 
## 5. Build & flash the firmware
 
> Do this in an SDK shell (see §4), from `firmware/`.
 
### Build
```powershell
west build -d build_2 -b nrf52840dk/nrf52840    # first time
west build -d build_2                            # subsequent builds (incremental)
```
 
### Flash — option A: J-Link (recommended for dev)
```powershell
west flash -d build_2
```
Console + logs on **RTT**: `JLinkRTTViewer` (or the RTT tab in nRF Connect).
 
### Flash — option B: USB DFU (no probe)
The native USB CDC port exposes mcumgr **serial recovery**. `upload` always writes to the
secondary slot: you then need `test` + `reset` to boot the image. The script automates everything:
```powershell
.\flash.ps1 -Port COM18       # adjust to the native CDC's COM
```
> ⚠️ Close the Python tool (§6) before a DFU: **one owner of the port** at a time.
> After reset, MCUboot waits ~30 s (DFU window) before launching the application.
 
### Configuration points to know (`prj.conf`)
- `CONFIG_UART_CONSOLE=n`, `CONFIG_LOG_BACKEND_UART=n` → **clean SMP channel** (logs on RTT).
- `CONFIG_MCUMGR_TRANSPORT_UART=y` → DFU + CV group over the CDC.
- `CONFIG_UDC_BUF_POOL_SIZE=16384`, `CONFIG_UDC_BUF_COUNT=32` → USB buffers (otherwise
  "udc: Failed to allocate net_buf").
---
 
## 6. Using the PC tool (`CV_python`)
 
```bash
cd CV_python
python main.py
```
 
Workflow in the UI:
1. **Detect** — probes each COM port with an SMP `STATUS` request and keeps the one that
   responds (= the DFU/mcumgr port, the same one as `flash.ps1`).
2. **Connect**.
3. Choose the **method** (see §7), set the parameters and the **RTIA range**.
4. **▶ Run** — the measurement runs on the board, the points are downloaded and plotted.
### Session / test & export
Enter a **session name** and a **test name** (+ free-form **notes** before the measurement).
For each saved measurement:
```
CV_python/mesures/<session>/<test>/<test>_<YYYYMMDD_HHMMSS>.csv
CV_python/mesures/<session>/<test>/<test>_<YYYYMMDD_HHMMSS>.txt   ← method, params, result, notes
```
CSV X column: `voltage_mV` (CV/DPV) or `time_ms` (CA).
 
---
 
## 7. Available techniques
 
| Method | id | X axis | Main parameters |
|--------|----|--------|-----------------|
| **Cyclic voltammetry (CV)** | 0 | Voltage (mV) | start, vertex, Vzero, step count, scan rate (mV/s), sample delay |
| **Chronoamperometry (i–t)** | 1 | Time (s)    | potential, Vzero, duration, interval |
| **DPV (differential pulse)** | 2 | Voltage (mV) | start, end, step height, pulse amplitude, pulse width, period |
 
- **RTIA** adjustable (table 200 Ω → 512 kΩ): sets the current window `I_max ≈ 0.9 V / RTIA`.
  **Auto-range** exists **only for CV**; for CA/DPV, choose a fixed range.
- CV = ADI sequencer engine (`RampTest.c`). CA & DPV = **software-timed** (direct LPDAC
  control + one-off ADC reads via `ad5940_lp.c`), current scale calibrated like CV.
---
 
## 8. SMP protocol (for firmware/PC development)
 
Custom group **`id 64`** (`MGMT_GROUP_ID_PERUSER`), transport = SMP over serial (the same as DFU).
 
| Cmd | Op | Input payload | Response |
|-----|----|---------------|----------|
| `START` (0)  | write | `method` + optional integer params (see below) | `{rc:0}` |
| `STATUS` (1) | read  | `{}` | `{run, n, peak_na, r_ohm, rtia, method}` |
| `DATA` (2)   | read  | `{off}` | `{off, n, v:[…], i:[…]}` (batches of 64) |
 
`START` keys (all optional, integers):
- **common**: `method` (0/1/2), `vzero_mv`, `rtia` (CV: `0`=auto-range; CA/DPV: `≤0`→4000)
- **CV**: `start_mv peak_mv steps dur_ms delay_us`
- **CA**: `ca_pot_mv ca_dur_ms ca_interval_ms`
- **DPV**: `start_mv end_mv dpv_estep_mv dpv_epulse_mv dpv_tpulse_ms dpv_tperiod_ms`
`v[]` = voltage mV (CV/DPV) **or** time ms (CA); `i[]` = current nA (ΔI for DPV).
 
Framing (implemented in `main.py`, identical to Zephyr's `serial_util.c`): marker
`0x0609` (first fragment) / `0x0414` (subsequent), base64 of `[len BE, 2 bytes][packet][CRC16 BE, 2 bytes]`,
terminated by `\n`; CRC16-ITU-T/XMODEM (poly `0x1021`, init `0`); SMP header `op,flags,len,group,seq,id`.
 
---
 
## 9. Extending
 
- **New technique**: create `XXX.c/.h` modeled on `CA.c` (uses `ad5940_lp`), add a value
  to `methods.h`, a `case` in `run_measurement()` (`main.c`), decode its keys in
  `cv_mgmt_start`, add it to `CMakeLists.txt`, then a parameters panel in `main.py`
  (`_build_*_params` + `_read_params`).
- **Current accuracy**: set `CV_RCAL_OHM` (`src/CV.h`) to the real RCAL.
---
 
## 10. Troubleshooting
 
| Symptom | Cause / fix |
|---------|-------------|
| Python "No board detected" | Wrong port, or port held by another program (mcumgr, serial terminal, duplicate GUI). Close the others, retry. |
| Port access denied / freeze | The DFU port is already open elsewhere — one owner at a time. |
| DFU "does not recognize the command" | Close the Python GUI before `flash.ps1`; check the right COM (native CDC, not the J-Link VCOM). |
| Measurement = 0 points / peak 0 | AD5940 silent (ADIID ≠ 0x4144): check SPI/power; or electrode wiring. See RTT. |
| Nothing on the serial console | Normal: logs on **RTT** (`CONFIG_UART_CONSOLE=n`). |
| `west`/`nrfutil` not found | Open an SDK shell (`nrfutil toolchain-manager launch --shell`) or VS Code + nRF Connect. |
| SWD/flash refused (APPROTECT) | `nrfutil device recover --serial-number <sn>` then `west flash`. |



# ParisBioTek — Firmware nRF52840 + AD5940 & outil PC de voltampérométrie

Plateforme de **mesure électrochimique** sur nRF52840 (Nordic) piloté depuis un PC :
un microcontrôleur exécute les techniques (voltampérométrie cyclique, chronoampérométrie,
DPV) sur un potentiostat **AD5940**, et un outil Python les déclenche / récupère / trace
via USB — **sur le même canal que la mise à jour du firmware (DFU)**.

> Ce dépôt couvre le **firmware** et l'**outil PC**. L'application Android compagnon
> (authentification NFC + BLE) n'est pas nécessaire pour développer ici.

---

## 1. Aperçu

```mermaid
flowchart LR
    subgraph PC
      A[CV_python/main.py<br/>Tkinter + matplotlib]
    end
    subgraph nRF52840
      B[Groupe SMP « CV » id 64] --> C[Dispatcher run_measurement]
      C --> D[CV.c / CA.c / DPV.c]
      D --> E[ad5940_port / ad5940_lp<br/>SPI]
    end
    subgraph Capteur
      F[AD5940 potentiostat] --> G[Électrodes CE0 / RE0 / SE0]
    end
    A <-->|USB CDC : SMP framing<br/>base64 + CRC16| B
    E <-->|SPI mode 0| F
```

- Un **seul port USB (CDC ACM)** sert à la fois au **DFU mcumgr** et au **pilotage des
  mesures** (groupe SMP custom `id 64`). Un seul programme peut ouvrir le port à la fois.
- Les **logs / console** sortent sur **RTT (J-Link)**, jamais sur l'USB (sinon les trames
  SMP seraient corrompues).

---

## 2. Arborescence

```
firmware/                     ← ce dossier (projet NCS/Zephyr, sysbuild + MCUboot)
├── CMakeLists.txt
├── prj.conf                  ← config Bluetooth/NFC/USB/mcumgr + RTT
├── app.overlay               ← 1 CDC ACM (console+DFU+CV), SPI1 AD5940, GPIO
├── flash.ps1                 ← flash USB DFU en 1 commande (upload→test→reset)
├── mcumgr-client.exe         ← client DFU série (0.0.9)
└── src/
    ├── main.c                ← USB/BLE/NFC + groupe SMP CV + dispatcher
    ├── methods.h             ← enum des techniques (CV=0, CA=1, DPV=2)
    ├── CV.c / CV.h           ← voltampérométrie cyclique (séquenceur AD5940)
    ├── CA.c / CA.h           ← chronoampérométrie (logiciel-timé)
    ├── DPV.c / DPV.h         ← voltamp. à impulsions différentielles (logiciel-timé)
    ├── ad5940_lp.c/.h        ← primitives boucle basse-puissance (LPDAC+LPTIA) + RTIA
    ├── RampTest.c/.h         ← moteur « ramp » ADI (utilisé par la CV)
    ├── ad5940.c/.h           ← pilote ADI AD5940
    └── ad5940_port.c/.h      ← portage SPI Zephyr + alimentation capteur

CV_python/                    ← outil PC (dossier voisin)
└── main.py                   ← GUI de pilotage / tracé / export
```

---

## 3. Matériel

| Élément            | Détail                                                              |
|--------------------|--------------------------------------------------------------------|
| Carte              | **nRF52840 DK** (`nrf52840dk/nrf52840`)                             |
| Capteur            | **AD5940** (potentiostat) sur **SPI1**, mode 0                      |
| Programmation      | **J-Link** (VCOM/RTT) *ou* **USB DFU** (mcumgr sur le CDC natif)    |
| Électrodes         | CE0 (contre-électrode), RE0 (référence), SE0 (travail)             |

Câblage SPI (voir `app.overlay`) : `SCLK=P0.13  MOSI=P0.22  MISO=P0.15  CS=P0.17`,
reset `P0.20`, alim capteur `P1.00`.

**Test « résistance »** (bring-up sans cellule réelle) : placer une résistance étalon
entre **CE0** et le nœud **(RE0 relié à SE0)**. Une CV doit alors donner une droite dont la
pente = la résistance. Le RTIA est calibré via une **RCAL externe** (`CV_RCAL_OHM` dans
`src/CV.h`) — ajuster cette valeur à la RCAL réellement câblée pour la précision.

---

## 4. Prérequis logiciels

### Firmware — nRF Connect SDK **v3.3.0**
Installer via le **Toolchain Manager** de Nordic (ou l'extension *nRF Connect for VS Code*) :
- SDK : `C:\ncs\v3.3.0`
- Toolchain : `C:\ncs\toolchains\936afb6332`

`west` / `nrfutil` ne sont pas sur le PATH global : ouvrir un shell SDK avec
```powershell
nrfutil toolchain-manager launch --shell
```
ou simplement **ouvrir le dossier `firmware/` dans VS Code + nRF Connect** (build/flash en clic).

### Outil PC — Python 3.9+
```bash
pip install pyserial cbor2 matplotlib
```
(`tkinter` est fourni avec Python sous Windows.)

---

## 5. Compiler & flasher le firmware

> À faire dans un shell SDK (voir §4), depuis `firmware/`.

### Build
```powershell
west build -d build_2 -b nrf52840dk/nrf52840    # 1re fois
west build -d build_2                            # builds suivants (incrémental)
```

### Flash — option A : J-Link (recommandé en dev)
```powershell
west flash -d build_2
```
Console + logs sur **RTT** : `JLinkRTTViewer` (ou l'onglet RTT de nRF Connect).

### Flash — option B : USB DFU (sans sonde)
Le port USB CDC natif expose le **serial recovery mcumgr**. `upload` écrit toujours dans le
slot secondaire : il faut ensuite `test` + `reset` pour booter l'image. Le script automatise tout :
```powershell
.\flash.ps1 -Port COM18       # adapter au COM du CDC natif
```
> ⚠️ Fermer l'outil Python (§6) avant un DFU : **un seul propriétaire du port** à la fois.
> Après reset, MCUboot attend ~30 s (fenêtre DFU) avant de lancer l'application.

### Points de configuration à connaître (`prj.conf`)
- `CONFIG_UART_CONSOLE=n`, `CONFIG_LOG_BACKEND_UART=n` → **canal SMP propre** (logs sur RTT).
- `CONFIG_MCUMGR_TRANSPORT_UART=y` → DFU + groupe CV sur le CDC.
- `CONFIG_UDC_BUF_POOL_SIZE=16384`, `CONFIG_UDC_BUF_COUNT=32` → buffers USB (sinon
  « udc: Failed to allocate net_buf »).

---

## 6. Utiliser l'outil PC (`CV_python`)

```bash
cd CV_python
python main.py
```

Flux dans l'interface :
1. **Détecter** — sonde chaque port COM avec une requête SMP `STATUS` et retient celui qui
   répond (= le port DFU/mcumgr, le même que `flash.ps1`).
2. **Connecter**.
3. Choisir la **méthode** (voir §7), régler les paramètres et la **gamme RTIA**.
4. **▶ Lancer** — la mesure s'exécute sur la carte, les points sont téléchargés et tracés.

### Session / test & export
Renseigne un **nom de session** et un **nom de test** (+ **notes** libres avant la mesure).
À chaque mesure enregistrée :
```
CV_python/mesures/<session>/<test>/<test>_<AAAAMMJJ_HHMMSS>.csv
CV_python/mesures/<session>/<test>/<test>_<AAAAMMJJ_HHMMSS>.txt   ← méthode, params, résultat, notes
```
Colonne X du CSV : `voltage_mV` (CV/DPV) ou `time_ms` (CA).

---

## 7. Techniques disponibles

| Méthode | id | Axe X | Paramètres principaux |
|---------|----|-------|-----------------------|
| **Voltampérométrie cyclique (CV)** | 0 | Tension (mV) | départ, sommet, Vzero, nb de pas, vitesse (mV/s), délai échantillon |
| **Chronoampérométrie (i–t)**       | 1 | Temps (s)    | potentiel, Vzero, durée, intervalle |
| **DPV (impulsions différentielles)** | 2 | Tension (mV) | départ, fin, hauteur de marche, amplitude d'impulsion, durée d'impulsion, période |

- **RTIA** réglable (table 200 Ω → 512 kΩ) : fixe la fenêtre de courant `I_max ≈ 0,9 V / RTIA`.
  L'**auto-range** n'existe **que pour la CV** ; en CA/DPV, choisir une gamme fixe.
- CV = moteur séquenceur ADI (`RampTest.c`). CA & DPV = **logiciel-timés** (pilotage direct du
  LPDAC + lectures ADC ponctuelles via `ad5940_lp.c`), échelle de courant calibrée comme la CV.

---

## 8. Protocole SMP (pour développer côté firmware/PC)

Groupe custom **`id 64`** (`MGMT_GROUP_ID_PERUSER`), transport = SMP sur série (le même que le DFU).

| Cmd | Op | Payload entrée | Réponse |
|-----|----|----------------|---------|
| `START` (0)  | write | `method` + params entiers optionnels (voir ci-dessous) | `{rc:0}` |
| `STATUS` (1) | read  | `{}` | `{run, n, peak_na, r_ohm, rtia, method}` |
| `DATA` (2)   | read  | `{off}` | `{off, n, v:[…], i:[…]}` (lots de 64) |

Clés `START` (toutes optionnelles, entiers) :
- **commun** : `method` (0/1/2), `vzero_mv`, `rtia` (CV : `0`=auto-range ; CA/DPV : `≤0`→4000)
- **CV** : `start_mv peak_mv steps dur_ms delay_us`
- **CA** : `ca_pot_mv ca_dur_ms ca_interval_ms`
- **DPV** : `start_mv end_mv dpv_estep_mv dpv_epulse_mv dpv_tpulse_ms dpv_tperiod_ms`

`v[]` = tension mV (CV/DPV) **ou** temps ms (CA) ; `i[]` = courant nA (ΔI en DPV).

Framing (implémenté dans `main.py`, identique à `serial_util.c` de Zephyr) : marqueur
`0x0609` (1er fragment) / `0x0414` (suivant), base64 de `[len BE 2o][paquet][CRC16 BE 2o]`,
terminé par `\n` ; CRC16-ITU-T/XMODEM (poly `0x1021`, init `0`) ; en-tête SMP `op,flags,len,group,seq,id`.

---

## 9. Étendre

- **Nouvelle technique** : créer `XXX.c/.h` sur le modèle de `CA.c` (utilise `ad5940_lp`),
  ajouter une valeur à `methods.h`, un `case` dans `run_measurement()` (`main.c`), décoder ses
  clés dans `cv_mgmt_start`, l'ajouter à `CMakeLists.txt`, puis un panneau de paramètres dans
  `main.py` (`_build_*_params` + `_read_params`).
- **Précision courant** : ajuster `CV_RCAL_OHM` (`src/CV.h`) à la RCAL réelle.

---

## 10. Dépannage

| Symptôme | Cause / solution |
|----------|------------------|
| Python « Aucune carte détectée » | Mauvais port, ou port tenu par un autre logiciel (mcumgr, terminal série, GUI en double). Fermer les autres, réessayer. |
| Accès au port refusé / freeze | Le port DFU est déjà ouvert ailleurs — un seul propriétaire à la fois. |
| DFU « ne reconnaît pas la commande » | Fermer le GUI Python avant `flash.ps1` ; vérifier le bon COM (CDC natif, pas la VCOM J-Link). |
| Mesure = 0 point / pic 0 | AD5940 muet (ADIID ≠ 0x4144) : vérifier SPI/alim ; ou câblage électrodes. Voir RTT. |
| Rien sur la console série | Normal : logs sur **RTT** (`CONFIG_UART_CONSOLE=n`). |
| `west`/`nrfutil` introuvable | Ouvrir un shell SDK (`nrfutil toolchain-manager launch --shell`) ou VS Code + nRF Connect. |
| SWD/flash refusé (APPROTECT) | `nrfutil device recover --serial-number <sn>` puis `west flash`. |

---

## 11. Notes / limites

- CA & DPV sont **logiciel-timés** (précision d'impulsion ≥ ~10–50 ms) — une version
  séquenceur (plus rapide/précise) reste à faire.
- L'**auto-range** est réservé à la CV.
- Le NFC/BLE (app Android) reste fonctionnel mais hors périmètre de ce README.
