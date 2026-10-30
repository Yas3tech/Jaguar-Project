# camera/ — camerabeeld en obstakeldetectie

Standaard uit. Aanzetten op de live pagina: **Opties → Camera** (live beeld) en **Opties → Obstakels** (vooruit stoppen
voor wat de camera ziet). Nodig: `pip install opencv-python numpy` (alleen voor de obstakels; het beeld zelf niet).

| Bestand | Wat |
|---|---|
| `obstakel.py` | Obstakeldetectie met OpenCV: zoekt per beeldkolom waar de vloer ophoudt en rekent dat om naar een afstand |
| `../../config/camera_kalibratie.json` | De kalibratiepunten van de Axis-camera (muur op 1, 2 en 3 m) |

De oude staat van vóór de mappenstructuur staat in git: branch `camera-origineel`.

## De camera zelf

Axis-camera op de robot, 192.168.0.65:8081, met Digest-login (zie `docs/robot/Notes.txt`). Beeld: MJPEG op
`/axis-cgi/mjpg/video.cgi?resolution=640x480&fps=10`, losse foto op `/axis-cgi/jpg/image.cgi`. `jaguar/live/view.py`
stuurt het beeld door naar de pagina (`/camera`, met de login), zodat de browser geen wachtwoord nodig heeft.

## Hoe het werkt

1. De camera staat vast op de robot, dus elke beeldrij hoort bij een vaste afstand op de vloer (`d = k / (y − y_horizon)`).
2. Eenmalig kalibreren (Opties → Obstakels → Kalibreren…) met de voet van een muur op minstens twee afstanden.
3. In de middelste strook van het beeld zoekt `obstakel.py` van onder naar boven de eerste plotse kleursprong: dat is het dichtste obstakel.
4. `jaguar/kern/robot.py` remt vooruit af en stopt op de stopafstand (standaard 0,4 m); geen vers beeld (1,5 s) = niet blind rijden (`CameraWeg`).
5. `jaguar/rijden/parcours.py` wacht tot 4 s, rijdt eromheen (45° opzij) of kort de stap in, en gaat dan verder met het parcours.
6. Handmatig rijden (`bediening.py`): vooruit is geblokkeerd zolang er iets voor de robot staat; draaien en achteruit niet.

Grenzen: alleen vooruit, en niet bij glas, een muur in de kleur van de vloer, een sterk vloerpatroon of in het donker.
De noodstop blijft nodig.

## Een eigen model aansluiten

Vervang de vloerlijn-detectie in `obstakel.py` door het model. `robot.py` gebruikt alleen een `obstacle`-object met
`latest()` (afstand in m of None, leeftijd van het beeld in s), `stop` (stopafstand in m) en `error`; `parcours.py`
daarnaast `dist` en `free_side(m)` voor de omweg.
