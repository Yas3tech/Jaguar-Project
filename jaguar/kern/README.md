# kern/ — de basis

- `robot.py`: verbinding met de robot (TCP), motoren, rijden op afstand, ter plaatse draaien, veiligheidsstops (kantelen, vastlopen, batterij, obstakel vooruit als de camera aan staat), pauze, het opslaan van de CSV en `robot_session()` waarmee elk rijscript begint. Alle rijlussen sturen de motoren via `_motor()`: daar werken STOP (`Aborted`), Pauze (`Gepauzeerd`), Terug naar start (`NaarStart`) en de batterij (`BatterijLaag`, `BatterijLeeg`) in elke fase.
- `odometrie.py`: rekent uit de vier wielencoders, de gyro en de accelerometer de afgelegde weg, koers en slip uit. Dezelfde code draait live tijdens het rijden en achteraf in de analyse.

Hier start je niets; de scripts in `rijden/` gebruiken dit.
