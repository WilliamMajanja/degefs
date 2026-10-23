PYTHON ?= python

.PHONY: run test compile bench

run:
	$(PYTHON) -m src

test:
	$(PYTHON) -m pytest -q

compile:
	$(PYTHON) -m compileall -q src degefs tests

bench:
	$(PYTHON) -m pytest -q tests/test_bench.py
