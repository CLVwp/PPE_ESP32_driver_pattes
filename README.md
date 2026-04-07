# PPE ESP32 — driver quadrupède

Firmware PlatformIO pour ESP32 : capteurs (I2C), 8 servos PWM, liaison **USB série** (JSON ligne par ligne).

## Liaison série

| Paramètre | Valeur                                                    |
| --------- | --------------------------------------------------------- |
| Débit     | **115200** baud                                           |
| Format    | **8N1**                                                   |
| Messages  | **une ligne JSON** terminée par **LF** (`\n`) ou **CRLF** |

Chaque ligne est un objet JSON indépendant (style **NDJSON**).

### Moniteur série depuis une Raspberry Pi

On utilise **`picocom`** (léger, sans `screen`) :

```bash
sudo apt install -y picocom
picocom -b 115200 /dev/ttyUSB0
```

Remplace **`/dev/ttyUSB0`** par le port réel (`ttyACM0`, `ttyUSB2`, etc. — voir `ls /dev/ttyUSB* /dev/ttyACM*`). En cas de _Permission denied_ : `sudo usermod -aG dialout $USER` puis reconnecte la session.

Quitter picocom : **Ctrl+A**, puis **Ctrl+X**.

---

## Commandes : hôte → ESP32

Toutes les commandes reconnues utilisent le champ **`"t"`** (`cmd` ou `srv`).

### `stand` — pose neutre

Pose **haute** : IK **pied sous l’épaule** avec fémur et tibia de **`LEG_LINK_LENGTH_MM`** (13 cm par défaut) : hauteur cible **`STAND_HEIGHT_MM`** = **2L** (~260 mm), soit patte **droite verticale**. Les angles logiques **neutres** **`NEUTRAL_SHOULDER_DEG` / `NEUTRAL_KNEE_DEG`** (90° / 45° par défaut) sont le calage « tout droit » ; les **trim** s’ajoutent après.

```json
{ "t": "cmd", "m": "stand" }
```

### `stand_low` — stand bas (corps près du sol)

`stand_low` est le **stand bas IK** (hauteurs par rangée) : **`STAND_LOW_HEIGHT_FRONT_ROW_MM`** et **`STAND_LOW_HEIGHT_REAR_ROW_MM`** dans `robot_config.h`. Les **trim** s’appliquent encore après.

```json
{ "t": "cmd", "m": "stand_low" }
```

### `stand_low_gorille` — stand bas calibré manuel

Pose directe en angles via **`STAND_LOW_GORILLA_DEG[]`** dans `robot_config.h` (FL..RR : épaule puis genou), issue de ton calage `srv`.

```json
{ "t": "cmd", "m": "stand_low_gorille" }
```

Tu peux aussi envoyer `stand_low_gorilla` (alias accepté).

Pour modifier ce preset, ajuste **`STAND_LOW_GORILLA_DEG[]`** puis recompile.

**Transitions** :

- `stand` ↔ `stand_low` : fondu IK en hauteur _H_ par patte (smoothstep, `POSE_BLEND_DURATION_MS`).
- vers/depuis `stand_low_gorille` : fondu dans l’espace des angles.

### `walk` — marche (trot)

Démarre la séquence de marche en boucle autour d’une **pose de base marche** (plus basse que le stand IK) : **`WALK_BASE_SHOULDER_FRONT_DEG` / `REAR`** et **`WALK_BASE_KNEE_FRONT_DEG` / `REAR`** dans `robot_config.h` (posture type « gorille » : avant vers l’avant, arrière vers l’arrière, genoux plus fléchis — à caler au banc).

Le paramètre **`v`** (optionnel) fixe un facteur de vitesse entre environ **0,05** et **1,0** ; s’il est absent, une valeur par défaut (~0,55) est utilisée côté parseur.

Anti-croisement côté même flanc : **`WALK_SHOULDER_INNER_*`**, **`WALK_SHOULDER_SWING_DEG`**, et **`WALK_KNEE_LIFT_INNER_SCALE`** (réduit le relevé du genou arrière quand l’épaule arrière est en phase intérieure).

```json
{ "t": "cmd", "m": "walk" }
```

```json
{ "t": "cmd", "m": "walk", "v": 0.5 }
```

### `walk_gorille` — marche depuis `stand_low_gorille`

Marche douce basée directement sur **`STAND_LOW_GORILLA_DEG[]`** (marges mécaniques restantes).  
Gait en **trot diagonal** (FL+RR / FR+RL) avec **IK 2D** : trajectoire du **pied** \((x,z)\) en mm depuis l’épaule, puis angles déduits (voir **`WALK_GORILLA_STRIDE_MM`**, **`CLEARANCE_MM`**, **`STANCE_SLIDE_MM`**, **`TURN_STRIDE_MM`** dans `robot_config.h`).

Paramètres :

- **`v`** : vitesse (0.1–1.0)
- **`x`** : translation (**-1** recule, **+1** avance)
- **`yaw`** : rotation (**-1** tourne gauche, **+1** tourne droite)

```json
{ "t": "cmd", "m": "walk_gorille", "v": 0.75, "x": 1, "yaw": 0 }
```

w

```json
{ "t": "cmd", "m": "walk_gorille", "v": 0.35, "x": -0.6, "yaw": 0 }
```

```json
{ "t": "cmd", "m": "walk_gorille", "v": 0.4, "x": 0.3, "yaw": 0.8 }
```

Tu peux aussi envoyer `walk_gorilla` (alias accepté).

### `motion` — direction à chaud (walk / walk_gorille)

Permet de changer avance/recule/rotation sans quitter le mode de marche courant.

```json
{ "t": "cmd", "m": "motion", "x": 0.8, "yaw": -0.4 }
```

### `speed` — vitesse en marche

À utiliser **pendant** le mode marche pour changer le facteur de vitesse (clamp côté firmware vers ~**0,1**–**1,0**).

```json
{ "t": "cmd", "m": "speed", "v": 1.0 }
```

### `srv` — un servo, angle absolu

- **`i`** : index du servo **0 à 7** (ordre **FL → FR → RL → RR**, épaule puis genou pour chaque patte)
- **`a`** : angle **logique** en **degrés**, **0 à 180** (borné côté firmware)

L’angle est dans un **repère robot symétrique** : sur les pattes **droites** (FR, RR), le firmware applique le **miroir PWM** `180° − a` pour un même sens mécanique qu’à gauche. Le **90° logique** reste **90°** en PWM sur les deux côtés.

```json
{ "t": "srv", "i": 1, "a": 60 }
```

**Important** : les clés sont bien **`i`** et **`a`** (pas des numéros en guise de noms de champs).

---

## Indices servos, GPIO et miroir (ordre logique)

**Changement important** : les indices **`i`** ne correspondent plus à l’ancien mélange avant/arrière dans l’ordre des canaux. Tout script ou séquence enregistrée avec l’ancien mapping doit être **réécrit**.

Convention : **vue de dessus**, tête vers l’avant — **FL** avant gauche, **FR** avant droite, **RL** arrière gauche, **RR** arrière droite. Pour chaque patte : **`2×patte` = épaule**, **`2×patte+1` = genou**.

| Index `i` | GPIO | Patte | Articulation | Miroir PWM (FR, RR) |
| --------- | ---- | ----- | ------------ | ------------------- |
| 0         | 33   | FL    | épaule       | non                 |
| 1         | 25   | FL    | genou        | non                 |
| 2         | 26   | FR    | épaule       | **oui**             |
| 3         | 32   | FR    | genou        | **oui**             |
| 4         | 13   | RL    | épaule       | non                 |
| 5         | 12   | RL    | genou        | non                 |
| 6         | 14   | RR    | épaule       | **oui**             |
| 7         | 27   | RR    | genou        | **oui**             |

Tables détaillées : `SERVO_PINS[]`, `SERVO_ANGLE_MIRROR[]`, **`SERVO_TRIM_DEG[]`**, constantes IK (**`LEG_LINK_LENGTH_MM`**, hauteurs stand / stand bas, gains genou) dans `src/robot_config.h`.

Les broches **GPIO 4 et 5** de la carte sont réservées au bus **I2C** des capteurs.

### Trim mécanique (`SERVO_TRIM_DEG[]`)

Chaque case **0–7** ajoute un **offset en degrés** à l’angle **logique** de ce servo **avant** le miroir et la PWM. Sert à corriger un horn mal centré ou des petites différences entre servos : avec la même consigne (`stand` ou huit fois `srv` à **90**), tu peux aligner visuellement les membres en mettant par exemple `2.f`, `-3.f`, etc.

- Ordre des indices : identique au tableau ci-dessus (FL épaule, FL genou, …).
- La **télémétrie** `robot.srv[]` affiche les angles **sans** trim (consigne interne) ; seule la sortie PWM applique `trim + miroir`.

Après réglage au banc, recompile pour garder les offsets.

---

## Trouver un stand stable

Les valeurs par défaut (**épaule 90°, genou 45°**) décrivent un **repère géométrique**, pas une pose garantie **stable** : masse, base d’appui et frottements jouent. À ajuster **au banc** :

1. Envoyer `{"t":"cmd","m":"stand"}`, robot au sol (ou léger support).
2. Observer basculement : souvent, **abaisser le corps** (modifier surtout les **genoux**, les quatre de la même façon pour garder la symétrie) améliore la stabilité.
3. Pour un **désalignement** qui se répète à **même consigne** sur un servo, préfère **`SERVO_TRIM_DEG[i]`** ; pour une **pose globale** (trop haut, trop bas), modifie **`NEUTRAL_*`** ou ajuste avec des `srv` sur les genoux **1, 3, 5, 7** puis épaules **0, 2, 4, 6**.
4. Quand la pose convient, reporter les angles dans **`NEUTRAL_SHOULDER_DEG`** et **`NEUTRAL_KNEE_DEG`** (et les trims dans **`SERVO_TRIM_DEG[]`**) puis recompiler pour que `stand` / `walk` restent cohérents.

Rappel : la projection verticale du centre de masse doit tomber **dans** le quadrilatère d’appui des pieds.

---

## Réponses : ESP32 → hôte

### Télémétrie (optionnelle)

Si `kSerialTelemetryOut` est à **`true`** dans `src/task_json.cpp`, l’ESP32 envoie environ **20 lignes/s** :

```json
{"t":"tel","seq":N,"imu":{...},"robot":{...}}
```

Contenu typique : accéléro, gyro, mag, baro, mode robot (`stand` / `stand_low` / `walk` / `pose`), phase de marche, vitesse, tableau des 8 angles **logiques** (même ordre que le tableau ci-dessus).

### Courant / charge par patte (servos classiques PWM)

Les servos **analogiques** en PWM (consigne d’angle seulement) **ne renvoient pas** le courant ni la charge au firmware : l’ESP32 ne peut donc **pas** savoir en temps réel « quelle patte supporte plus de poids » sans **matériel supplémentaire**.

Pistes possibles côté robot :

- **Shunt + INA219 / INA226** (I2C) sur la **branche d’alimentation** de chaque patte ou de chaque paire (mesure de courant, pas de couple direct).
- **Servos « intelligents »** (bus série type Dynamixel / Feetech STS) avec télémétrie courant/torque si le fabricant l’expose — autre firmware et câblage.
- **Plateforme de pesée** (jauge / capteur de force) sous chaque pied — hors scope de ce dépôt.

Pour une posture **plus basse** et des pieds **plus près du sol** en marche : augmente **`WALK_BASE_KNEE_*`** (plus fléchi) et réduis **`WALK_KNEE_LIFT_DEG`** (moins de décollage en phase de trot). Rappel : en **trot diagonal**, deux pattes en diagonale sont en phase d’**oscillation** : un léger décollage est normal ; pour **quatre appuis permanents**, il faudrait un autre gabarit de marche (ex. crawl), non implémenté ici.

### Accusés (quand la télémétrie est désactivée)

Avec `kSerialTelemetryOut == false`, chaque commande acceptée ou refusée peut produire une ligne courte :

| Ligne                         | Signification                                                   |
| ----------------------------- | --------------------------------------------------------------- |
| `{"t":"ack","m":"stand"}`     | Commande `cmd` / `stand` prise en compte                        |
| `{"t":"ack","m":"stand_low"}` | Commande `cmd` / `stand_low` prise en compte                    |
| `{"t":"ack","m":"walk"}`      | Commande `cmd` / `walk` prise en compte                         |
| `{"t":"ack","m":"speed"}`     | Commande `cmd` / `speed` prise en compte                        |
| `{"t":"ack","m":"srv"}`       | Commande `srv` prise en compte                                  |
| `{"t":"ack","err":"json"}`    | Ligne JSON invalide                                             |
| `{"t":"ack","err":"ignored"}` | Message ignoré (mauvais `t`, `i` hors plage, file pleine, etc.) |

Les lignes qui **ne commencent pas par `{`** sont ignorées **sans** réponse.

---

## Exemples d’enchaînement

```text
{"t":"cmd","m":"stand"}
{"t":"srv","i":1,"a":55}
{"t":"cmd","m":"walk","v":0.35}
{"t":"cmd","m":"stand"}
```

### Forcer tous les canaux au même angle logique (test)

Pour mettre **tous** les servos à la même consigne **logique** (≠ pose `stand` qui utilise épaule/genou différents) :

```text
{"t":"srv","i":0,"a":90}
{"t":"srv","i":1,"a":90}
{"t":"srv","i":2,"a":90}
{"t":"srv","i":3,"a":90}
{"t":"srv","i":4,"a":90}
{"t":"srv","i":5,"a":90}
{"t":"srv","i":6,"a":90}
{"t":"srv","i":7,"a":90}
```

### Tous les servos à 0° ou 180° (test / butées)

**Attention mécanique** : vérifier que la patte ne force pas (risque d’abîmer les engrenages).

**Tous à 0° :**

```text
{"t":"srv","i":0,"a":0}
{"t":"srv","i":1,"a":0}
{"t":"srv","i":2,"a":0}
{"t":"srv","i":3,"a":0}
{"t":"srv","i":4,"a":0}
{"t":"srv","i":5,"a":0}
{"t":"srv","i":6,"a":0}
{"t":"srv","i":7,"a":0}
```

**Tous à 180° :**

```text
{"t":"srv","i":0,"a":180}
{"t":"srv","i":1,"a":180}
{"t":"srv","i":2,"a":180}
{"t":"srv","i":3,"a":180}
{"t":"srv","i":4,"a":180}
{"t":"srv","i":5,"a":180}
{"t":"srv","i":6,"a":180}
{"t":"srv","i":7,"a":180}
```

** LOW GROUND POSE**
{ "t": "cmd", "m": "stand_low" }

{"t":"srv","i":0,"a":140} épaule 2
{"t":"srv","i":1,"a":150} genou 1
{"t":"srv","i":2,"a":130} epaule 1
{"t":"srv","i":3,"a":150} genou 2

{"t":"srv","i":4,"a":30} epaule 3
{"t":"srv","i":5,"a":160} genou 3
{"t":"srv","i":6,"a":30} eppaule 4
{"t":"srv","i":7,"a":160} genou 4

** gorille low ground **
{"t":"srv","i":0,"a":150} épaule 2
{"t":"srv","i":1,"a":80} genou 1
{"t":"srv","i":2,"a":150} epaule 1
{"t":"srv","i":3,"a":80} genou 2

{"t":"srv","i":4,"a":150} epaule 3
{"t":"srv","i":5,"a":0} genou 3
{"t":"srv","i":6,"a":150} eppaule 4
{"t":"srv","i":7,"a":0} genou 4

{ "t":"cmd", "m":"stand_low_gorille" }
{ "t":"cmd", "m":"walk_gorille", "v":0.45, "x":1, "yaw":0 }
{ "t":"cmd", "m":"motion", "x":0.3, "yaw":0.8 }
{ "t":"cmd", "m":"motion", "x":-0.6, "yaw":0 }

---

## Moniteur série et envoi au clavier

Le panneau moniteur de certains IDE ne **transmet pas** le clavier vers le port. Utiliser un terminal avec **`pio device monitor`** (voir `platformio.ini` : `monitor_echo`, `monitor_filters`) ou la tâche VS Code prévue dans `.vscode/tasks.json`.

---

## Compilation / flash

Projet **PlatformIO**, environnement `az-delivery-devkit-v4` (voir `platformio.ini`).

```bash
pio run -t upload
pio device monitor -b 115200
```

---

## Fichiers utiles

| Fichier               | Rôle                                                               |
| --------------------- | ------------------------------------------------------------------ |
| `src/task_json.cpp`   | Protocole série, parse des commandes, télémétrie / ack             |
| `src/task_servos.cpp` | PWM LEDC, modes stand / walk / pose                                |
| `src/robot_config.h`  | Broches, neutres stand, miroir, **trim**, amplitude marche, PWM µs |

##

picocom -b 115200 /dev/ttyUSBx

{ "t": "cmd", "m": "stand_low" }
{ "t": "cmd", "m": "stand_low_gorille" }
{ "t": "cmd", "m": "walk_gorille", "v": 0.65, "x": 0.9, "yaw": 0 }
{ "t": "cmd", "m": "motion", "x": 0.8, "yaw": -0.4 }