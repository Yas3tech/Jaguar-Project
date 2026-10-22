# Camera en obstakeldetectie (nog niet in gebruik)

Deze map staat bewust los van de rest van het project: niets in `jaguar/` importeert iets uit `camera/`.
De camera komt er pas aan het einde bij, met een eigen model.

## Wat hier staat

| Bestand | Wat |
|---|---|
| `obstakel.py` | Obstakeldetectie met OpenCV: zoekt per beeldkolom waar de vloer ophoudt en rekent dat om naar een afstand |
| `camera_kalibratie.json` | De kalibratiepunten van de Axis-camera (muur op 1, 2 en 3 m) |
| `integratie.patch` | Alles wat vroeger in de rest van de code hing: camerabeeld doorsturen op de live pagina, kalibratievenster, obstakelopties, afremmen en omweg in `parcours`, STOP bij obstakel |

De volledige oude staat staat ook in git: `git show camera-origineel` (branch `camera-origineel`).

## De camera zelf

Axis-camera op de robot, 192.168.0.65:8081, met Digest-login (zie `docs/robot/Notes.txt`). Beeld: MJPEG op
`/axis-cgi/mjpg/video.cgi?resolution=640x480&fps=10`, losse foto op `/axis-cgi/jpg/image.cgi`.

## Hoe de oude versie werkte

1. De camera staat vast op de robot, dus elke beeldrij hoort bij een vaste afstand op de vloer (`d = k / (y − y_horizon)`).
2. Eenmalig kalibreren met de voet van een muur op minstens twee afstanden.
3. In de middelste strook van het beeld zoekt `obstakel.py` van onder naar boven de eerste plotse kleursprong: dat is het dichtste obstakel.
4. `robot.py` remde vooruit af en stopte op de stopafstand (0,4 m); geen vers beeld = niet blind rijden.
5. `parcours.py` wachtte tot 4 s, reed eromheen (45° opzij) of kortte de stap in.

Grenzen: alleen vooruit, en niet bij glas, een muur in de kleur van de vloer, een sterk vloerpatroon of in het donker.

## Later aansluiten

Met een eigen model vervangt je de vloerlijn-detectie in `obstakel.py` door het model. Het aansluitpunt in `jaguar/robot.py`
is klein: een `obstacle`-object met `latest()` (afstand en leeftijd van het beeld) en `stop`. Zie `integratie.patch` voor
de plekken waar dat vroeger aanhaakte (`git apply --check camera/integratie.patch` werkt niet meer rechtstreeks omdat de
bestanden naar `jaguar/` verhuisd zijn; pas de paden aan of lees het als voorbeeld).
