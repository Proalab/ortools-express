
import json
import sys

data = json.loads(sys.argv[1])

print(json.dumps(data))