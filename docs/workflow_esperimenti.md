# Esecuzione degli esperimenti e post-processing dei dati

**Guida operativa — PeopleFlow / HRISim**

Questa guida descrive la procedura completa per eseguire una simulazione, registrarne i dati su rosbag e produrre il dataset CSV finale. Per la definizione del modello di rischio e delle metriche estratte si rimanda a [risk_model.md](risk_model.md).

---

## 1. Panoramica del flusso

```
┌─────────────┐    ┌──────────────┐    ┌───────────────┐    ┌─────────────┐
│ Configura   │ →  │ Esegui       │ →  │ Metti in salvo│ →  │ Post-process│
│ parametri   │    │ (tstart)     │    │ il bag        │    │ (CSV)       │
└─────────────┘    └──────────────┘    └───────────────┘    └─────────────┘
   tmule yaml        registrazione        mv in ~/shared      process_bag.sh
                     automatica,
                     stop a MAX_DURATION
```

Ogni esecuzione produce **un** rosbag; il post-processing trasforma ogni bag in **un** CSV. Le esecuzioni vanno fatte una alla volta.

## 2. Prerequisiti

Container Docker in esecuzione e shell al suo interno:

```bash
./docrun.sh        # solo la prima volta / se il container non è attivo
./docshell.sh      # apre una shell nel container HRISim
```

I sorgenti sono montati dal host, quindi le modifiche fatte fuori dal container sono immediatamente visibili all'interno (i pacchetti C++ richiedono `catkin build`, gli script Python no).

## 3. Configurazione dei parametri

I parametri della sessione si impostano in `hrisim_tmule/tmule/hrisim_bringup.yaml` (oppure esportando la variabile d'ambiente prima di `tstart`, che ha la precedenza sul default). I principali per gli esperimenti:

| Variabile | Default | Significato |
|---|---:|---|
| `SCENARIO` | `scenario_tesi_2` | scenario pedsim da caricare (e nome del bag prodotto) |
| `MAX_AGENT_SPEED` | `0.0` | velocità dei pedoni in m/s; `0` = velocità casuale per agente $\mathcal{N}(1.34, 0.26)$ |
| `MAX_DURATION` | `330` | durata della registrazione in secondi; il bag si chiude da solo allo scadere |
| `ROBOT_MODE` | `1` | modalità di controllo del robot (0 CONTROLLED, 1 TELEOPERATION, 2 SOCIAL) |

Esempio di configurazione al volo, senza modificare lo yaml:

```bash
export MAX_AGENT_SPEED=2.0
export MAX_DURATION=600
tstart
```

### 3.1 Coerenza del soggetto (scenari speculari)

Gli esperimenti prevedono due configurazioni: **umano al centro** (`subject = 0`) o **robot al centro** (`subject = 1`). Il valore del soggetto deve essere coerente in due punti:

1. **finestra `risk` del tmule** — `roslaunch hrisim_risk risk.launch subject:=1`: determina sia il riferimento del calcolo del rischio, sia quale agente il `PedsimBridge` blocca a `WP_CENTER`;
2. **post-processing** — secondo argomento di `process_bag.sh` (Sezione 6).

Un disallineamento tra i due produce un dataset in cui il rischio è riferito all'agente sbagliato.

### 3.2 Velocità degli agenti a runtime (opzionale)

Con la simulazione in esecuzione, la velocità dei pedoni può essere modificata senza riavvii:

```bash
rosrun dynamic_reconfigure dynparam set /pedsim_simulator max_agent_speed 1.5
# oppure con GUI: rosrun rqt_reconfigure rqt_reconfigure  →  pedsim_simulator
```

Nota: se la velocità cambia durante una registrazione, il bag conterrà tratti a velocità diverse; per esperimenti controllati impostare la velocità **prima** di avviare la sessione.

## 4. Esecuzione

```bash
tstart     # avvia la sessione tmule (Gazebo, rviz, pedsim, risk, recording, ...)
tshow      # (opzionale) visualizza i pannelli della sessione
```

All'avvio della finestra `rec`, `rosbag record` inizia automaticamente a registrare i topic:

`/map`, `/hri/risk`, `/pedsim_simulator/simulated_agents`, `/tf`, `/tf_static`

nel file:

```
/home/hrisim/ros_ws/<SCENARIO>.bag        # es. /home/hrisim/ros_ws/scenario_tesi_2.bag
```

La registrazione **si arresta da sola** dopo `MAX_DURATION` secondi (`rosbag record --duration`). Al termine dell'esperimento:

```bash
tstop      # chiude l'intera sessione; un'eventuale registrazione ancora attiva viene finalizzata correttamente
```

È possibile interrompere prima di `MAX_DURATION`: il bag resta valido, semplicemente più corto.

## 5. Gestione dei bag

Il nome del bag è fisso (`<SCENARIO>.bag`): **una nuova esecuzione con lo stesso scenario sovrascrive il bag precedente**. Dopo ogni run, mettere in salvo il file — la cartella `~/shared` del container corrisponde a `./shared` della repository sul host:

```bash
mv ~/ros_ws/scenario_tesi_2.bag ~/shared/scenario_tesi_2_run01.bag
```

Convenzione suggerita: `<scenario>_<condizione>_runNN.bag` (es. `scenario_tesi_2_v2.0_run03.bag`), annotando le condizioni sperimentali (velocità, soggetto, durata).

## 6. Post-processing

Il post-processing riproduce il bag e ricalcola le metriche con la stessa `compute_risk` del nodo online (vedi [risk_model.md](risk_model.md), Sezione 5). **Va eseguito a simulazione ferma** (dopo `tstop`): lo script avvia un proprio `roscore` e ripubblica i topic registrati, che entrerebbero in conflitto con una sessione attiva.

```bash
roscd hrisim_analytics
./process_bag.sh <percorso_bag> [subject] [max_duration]
```

| Argomento | Default | Significato |
|---|---:|---|
| `percorso_bag` | — | percorso del file `.bag` da processare |
| `subject` | `0` | soggetto al centro: `0` umano, `1` robot — deve combaciare con la run registrata |
| `max_duration` | `330` | secondi di dati da estrarre — allinearlo alla `MAX_DURATION` usata in registrazione |

Esempio (robot al centro, run da 10 minuti):

```bash
./process_bag.sh ~/shared/scenario_tesi_2_run01.bag 1 600
```

Lo script: avvia `roscore`, imposta `use_sim_time`, lancia l'estrattore (`extract.launch`), riproduce il bag con `--clock` e, al termine, salva il CSV chiudendo i processi.

### 6.1 Output

Il CSV viene scritto in:

```
hrisim_analytics/data/<nome_bag>.csv      # visibile sul host in HRISim_docker/src/HRISim/hrisim_analytics/data/
```

Campionamento a 10 Hz, colonne:

| Colonna | Descrizione |
|---|---|
| `Timestamp` | tempo di simulazione (s) |
| `Vr` | modulo della velocità del robot (m/s), con rumore |
| `Vh` | modulo della velocità dell'umano (m/s), con rumore |
| `Risk` | rischio $\in [0,1]$ riferito al soggetto (nuova formulazione) |

Le colonne di velocità sono sempre riferite allo stesso agente (`Vr` robot, `Vh` umano), indipendentemente da chi ricopre il ruolo di soggetto; è il parametro `subject` a stabilire a chi è riferito il rischio.

Trasformazioni applicate dall'estrattore:

- **rumore gaussiano** su posizioni e velocità ($\sigma = 0.05$) prima del ricalcolo del rischio, per simulare l'incertezza di percezione;
- **shift markoviani**: `Risk` ritardato di 1 campione, la velocità del **soggetto** (`Vr` se `subject=1`, `Vh` se `subject=0`) di 2 campioni (allineamento causale delle serie);
- **trim** di 2 righe in testa e in coda (rimozione dei NaN introdotti dagli shift).

## 7. Checklist rapida per una run

1. ☐ parametri impostati (`SCENARIO`, `MAX_AGENT_SPEED`, `MAX_DURATION`);
2. ☐ `subject` coerente nella finestra `risk` del tmule (0 umano / 1 robot al centro);
3. ☐ eventuale bag precedente messo in salvo (verrà sovrascritto);
4. ☐ `tstart`, attesa fine registrazione (o `tstop` anticipato);
5. ☐ `tstop` e spostamento del bag in `~/shared` con nome descrittivo;
6. ☐ `./process_bag.sh <bag> <subject> <max_duration>` a sessione ferma;
7. ☐ verifica del CSV in `hrisim_analytics/data/`.

## 8. Problemi noti e rimedi

| Sintomo | Causa probabile | Rimedio |
|---|---|---|
| il bag manca o è vuoto | finestra `rec` non partita o run più corta dell'attesa | controllare il pannello `rec` con `tshow` |
| il CSV si ferma a 330 s ma la run era più lunga | `max_duration` non passato a `process_bag.sh` | ripetere il post-processing con il terzo argomento |
| rischio riferito all'agente sbagliato | `subject` incoerente tra tmule e post-processing | verificare Sezione 3.1 e riprocessare il bag |
| l'agente al centro non parte mai | nodo `risk` non attivo o `subject` errato | controllare il pannello `risk` e il flag `collision` su `/hri/risk` |
| `process_bag.sh` dà errori di roscore | eseguito con la simulazione attiva | fare `tstop` e rilanciare |
