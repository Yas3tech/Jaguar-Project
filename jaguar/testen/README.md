# testen/ — zonder robot oefenen

- `nep_robot.py`: speelt een opgenomen rit (`testdata/demo_rit.csv`) af alsof het de robot is.

```
python -m jaguar.testen.nep_robot
python -m jaguar.rijden.rechtdoor --ip 127.0.0.1 --duration 5 --no-csv
```
De nep-robot speelt altijd dezelfde opname af, dus rijden op afstand reageert niet echt.
- `sim_robot.py`: een gesimuleerde robot die wel op de motoren reageert (vertraging, wrijving, slip bij draaien, watchdog en E-Stop zoals de echte), om rijden, pauzeren, terug naar de start en handmatig rijden te proberen. Leerwaarden (`config/leerwaarden.json`) worden met 127.0.0.1 niet gelezen of bewaard.
```
python -m jaguar.testen.sim_robot
python -m jaguar.rijden.bediening --ip 127.0.0.1
python -m jaguar.testen.sim_robot --batterij 22.4 --verbruik 3      # batterij die snel leegloopt
```
- `test_navigatie.py`: controles voor de GPS-omrekening, het noorden en het anker, zonder robot (`python -m jaguar.testen.test_navigatie`).
- `test_bediening.py`: bediening.py tegen de gesimuleerde robot: pauze, terug naar start (tijdens en na de rit), lege batterij, handmatig rijden met de dodemansknop en STOP tijdens een pauze (`python -m jaguar.testen.test_bediening`, ongeveer twee minuten, geen CSV's).
- `sim_boog.py`: simulatie van een skid-steer robot om de boogregeling (`Jaguar.arc`) te controleren (`python -m jaguar.testen.sim_boog`).
