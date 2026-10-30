# config/

- `kalibratie.json`: de gemeten wielstraal. Wordt geschreven door `python -m jaguar.kalibratie.kalibreer`. Verwijder het bestand om terug te gaan naar de standaardwaarde (0,135 m).
- `leerwaarden.json`: de bijgeleerde regelwaarden (vermogen→snelheid, kruipstoot, draaistoot); geschreven na elke rit.
- `camera_kalibratie.json`: de kalibratiepunten van de camera (voet van een muur op 1, 2 en 3 m), voor de obstakeldetectie. Geschreven door Opties → Obstakels → Kalibreren op de live pagina.
