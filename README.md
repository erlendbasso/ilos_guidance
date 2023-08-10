# ILOS



get parameters:

```curl http://localhost:8000/ilos/params```

set parameters

```curl -X PUT -H "content-type:application/json" -d '{"proportional_gain": 2.0, "integral_gain": 0.5}' http://localhost:8000/ilos/params```