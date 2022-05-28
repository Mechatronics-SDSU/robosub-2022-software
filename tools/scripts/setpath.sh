cd ..
cd ..
export PYTHONPATH=""
export PYTHONPATH="$(pwd)"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src/gui"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src/gui/img"
echo "PYTHONPATH set to $PYTHONPATH"
