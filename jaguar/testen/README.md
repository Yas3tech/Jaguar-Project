# testen/ — zonder robot oefenen

- `nep_robot.py`: speelt een opgenomen rit (`testdata/demo_rit.csv`) af alsof het de robot is.

```
python -m jaguar.testen.nep_robot
python -m jaguar.rijden.rechtdoor --ip 127.0.0.1 --duration 5 --no-csv
```
De nep-robot speelt altijd dezelfde opname af, dus rijden op afstand reageert niet echt.
- `test_navigatie.py`: controles voor de GPS-omrekening, het noorden en het anker, zonder robot (`python -m jaguar.testen.test_navigatie`).
- `sim_boog.py`: simulatie van een skid-steer robot om de boogregeling (`Jaguar.arc`) te controleren (`python -m jaguar.testen.sim_boog`).
