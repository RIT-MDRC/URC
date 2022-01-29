import json

data = {}
data['moveRelativeX'] = [10]
json_data = json.dumps(data)

print(json_data)