cd ..
cd ..
export PYTHONPATH=""
export PYTHONPATH="$(pwd)"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src/gui"
echo "PYTHONPATH set to $PYTHONPATH"
