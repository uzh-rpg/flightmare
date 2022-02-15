python3 -m black flightrl
python3 -m isort flightrl 
python3 -m pylint flightrl 
python3 -m pydocstyle --convention=google flightrl 
python3 -m mypy --install-types --strict flightrl 