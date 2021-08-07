
import json
import sys

data = json.loads(sys.argv[1])

data['id'] = 10

print(json.dumps(data))