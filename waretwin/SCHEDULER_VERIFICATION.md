# Scheduler / Orders verification

## Static checks

- Python backend `compileall`: PASS
- Frontend TS/TSX syntax transpile: PASS (46 files)
- Scheduler frontend REST paths vs Django URL routes: PASS
- Migration chain: `0003_warehouse_map -> 0004_scheduler_orders_workpoints`

## Simulation execution checks

Pure SimEngine checks were executed without Django dependencies:

1. `external_scheduler=True` suppresses random demo task generation.
2. Scheduler-style task `INBOUND-1 -> OUTBOUND-1` assigned to R01 completed via A*.
3. Multi-leg route `INBOUND-1 -> SORT-01 -> OUTBOUND-1` completed in the requested order on the same robot.
4. Low-battery scheduler transfer does not create an orphan legacy task; the persistent scheduler owns failure/re-plan state.

## Preserved behavior

What-if clones explicitly re-enable autonomous simulation scheduling so the existing What-if feature continues to work even though the live/mock runtime is scheduler-authoritative.

## Environment limitation

The execution sandbox cannot download/install the project Django/npm dependencies from the Internet, so full `manage.py test` and browser/Vite E2E were not executed here. The project includes `backend/twin/tests/test_scheduler.py` for running on the target machine after `pip install -r requirements.txt`.
