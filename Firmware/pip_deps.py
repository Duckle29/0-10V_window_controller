Import ("env")

try:
    import pymcuprog
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pymcuprog")
