# PPE ESP32 — driver quadrupède

Firmware PlatformIO pour ESP32 : capteurs (I2C), 8 servos PWM, liaison **USB série** (JSON ligne par ligne).

## Liaison série

| Paramètre | Valeur |
|-----------|--------|
| Débit | **115200** baud |
| Format | **8N1** |
| Messages | **une ligne JSON** terminée par **LF** (`\n`) ou **CRLF** |

Chaque ligne est un objet JSON indépendant (style **NDJSON**).

---

## Commandes : hôte → ESP32

Toutes les commandes reconnues utilisent le champ **`"t"`** (`cmd` ou `srv`).

### `stand` — pose neutre

Met les **quatre pattes** sur les angles définis dans `src/robot_config.h` : **`NEUTRAL_SHOULDER_DEG`** (épaule) et **`NEUTRAL_KNEE_DEG`** (genou). Par défaut : **90° / 45°** (point de départ à affiner au banc — voir plus bas).

```json
{"t":"cmd","m":"stand"}
```

### `walk` — marche (trot)

Démarre la séquence de marche en boucle autour des mêmes neutres. Le paramètre **`v`** (optionnel) fixe un facteur de vitesse entre environ **0,05** et **1,0** ; s’il est absent, une valeur par défaut (~0,55) est utilisée côté parseur.

```json
{"t":"cmd","m":"walk"}
```

```json
{"t":"cmd","m":"walk","v":0.4}
```

### `speed` — vitesse en marche

À utiliser **pendant** le mode marche pour changer le facteur de vitesse (clamp côté firmware vers ~**0,1**–**1,0**).

```json
{"t":"cmd","m":"speed","v":0.6}
```

### `srv` — un servo, angle absolu

- **`i`** : index du servo **0 à 7** (ordre **FL → FR → RL → RR**, épaule puis genou pour chaque patte)
- **`a`** : angle **logique** en **degrés**, **0 à 180** (borné côté firmware)

L’angle est dans un **repère robot symétrique** : sur les pattes **droites** (FR, RR), le firmware applique le **miroir PWM** `180° − a` pour un même sens mécanique qu’à gauche. Le **90° logique** reste **90°** en PWM sur les deux côtés.

```json
{"t":"srv","i":1,"a":60}
```

**Important** : les clés sont bien **`i`** et **`a`** (pas des numéros en guise de noms de champs).

---

## Indices servos, GPIO et miroir (ordre logique)

**Changement important** : les indices **`i`** ne correspondent plus à l’ancien mélange avant/arrière dans l’ordre des canaux. Tout script ou séquence enregistrée avec l’ancien mapping doit être **réécrit**.

Convention : **vue de dessus**, tête vers l’avant — **FL** avant gauche, **FR** avant droite, **RL** arrière gauche, **RR** arrière droite. Pour chaque patte : **`2×patte` = épaule**, **`2×patte+1` = genou**.

| Index `i` | GPIO | Patte | Articulation | Miroir PWM (FR, RR) |
|-----------|------|-------|--------------|---------------------|
| 0 | 33 | FL | épaule | non |
| 1 | 25 | FL | genou | non |
| 2 | 26 | FR | épaule | **oui** |
| 3 | 32 | FR | genou | **oui** |
| 4 | 13 | RL | épaule | non |
| 5 | 12 | RL | genou | non |
| 6 | 14 | RR | épaule | **oui** |
| 7 | 27 | RR | genou | **oui** |

Tables détaillées : `SERVO_PINS[]` et `SERVO_ANGLE_MIRROR[]` dans `src/robot_config.h`.

Les broches **GPIO 4 et 5** de la carte sont réservées au bus **I2C** des capteurs.

---

## Trouver un stand stable

Les valeurs par défaut (**épaule 90°, genou 45°**) décrivent un **repère géométrique**, pas une pose garantie **stable** : masse, base d’appui et frottements jouent. À ajuster **au banc** :

1. Envoyer `{"t":"cmd","m":"stand"}`, robot au sol (ou léger support).
2. Observer basculement : souvent, **abaisser le corps** (modifier surtout les **genoux**, les quatre de la même façon pour garder la symétrie) améliore la stabilité.
3. Ajuster par petits pas (2–5°) avec des `srv` sur les indices **1, 3, 5, 7** (genoux) puis si besoin **0, 2, 4, 6** (épaules).
4. Quand la pose convient, reporter les angles dans **`NEUTRAL_SHOULDER_DEG`** et **`NEUTRAL_KNEE_DEG`** dans `robot_config.h` et recompiler pour que `stand` / `walk` s’y calent.

Rappel : la projection verticale du centre de masse doit tomber **dans** le quadrilatère d’appui des pieds.

---

## Réponses : ESP32 → hôte

### Télémétrie (optionnelle)

Si `kSerialTelemetryOut` est à **`true`** dans `src/task_json.cpp`, l’ESP32 envoie environ **20 lignes/s** :

```json
{"t":"tel","seq":N,"imu":{...},"robot":{...}}
```

Contenu typique : accéléro, gyro, mag, baro, mode robot (`stand` / `walk` / `pose`), phase de marche, vitesse, tableau des 8 angles **logiques** (même ordre que le tableau ci-dessus).

### Accusés (quand la télémétrie est désactivée)

Avec `kSerialTelemetryOut == false`, chaque commande acceptée ou refusée peut produire une ligne courte :

| Ligne | Signification |
|-------|----------------|
| `{"t":"ack","m":"stand"}` | Commande `cmd` / `stand` prise en compte |
| `{"t":"ack","m":"walk"}` | Commande `cmd` / `walk` prise en compte |
| `{"t":"ack","m":"speed"}` | Commande `cmd` / `speed` prise en compte |
| `{"t":"ack","m":"srv"}` | Commande `srv` prise en compte |
| `{"t":"ack","err":"json"}` | Ligne JSON invalide |
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

| Fichier | Rôle |
|---------|------|
| `src/task_json.cpp` | Protocole série, parse des commandes, télémétrie / ack |
| `src/task_servos.cpp` | PWM LEDC, modes stand / walk / pose |
| `src/robot_config.h` | Broches, neutres stand, miroir, amplitude marche, PWM µs |
