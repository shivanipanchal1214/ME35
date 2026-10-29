import requests 
import json


# Assemble the URL for the API call 
api_url = "https://cad.onshape.com/api/v17/documents/774d39bdec33d0c04457313f"

# Optional query parameters can be assigned 
params = {}

# Use your keys
access_key = "on_aYJnrIVMwdIYUeYDZvrYE"
secret_key = "8Ack1ZdfWlLaNf7Q4E4mQLOtHIPOi2uC4VhfZAdUZX9RkLKd"

# Define the header for the request 
headers = {'Accept': 'application/json;charset=UTF-8;qs=0.09',
           'Content-Type': 'application/json'}

# Putting everything together to make the API request 
response = requests.get(api_url, 
                        params=params, 
                        auth=(access_key, secret_key),
                        headers=headers)

# Convert the response to formatted JSON and print the `name` property
#print(json.dumps(response.json()["name"], indent=4))

print("Status code:", response.status_code)
print("Content type:", response.headers.get("Content-Type"))
print("Response:", response.text)