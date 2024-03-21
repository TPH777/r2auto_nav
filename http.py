import requests

def make_http_post_request(ip_address):
    url = f'http://{ip_address}/openDoor'
    headers = {'Content-Type': 'application/json'}
    data = '{"action": "openDoor", "parameters": {"robotId": "32"}}'
    
    door_value = "No door information available"

    try:
        response = requests.post(url, headers=headers, data=data)
        print('Status Code:', response.status_code)
        
        response_data = response.json()
        door_value = response_data.get('data', {}).get('message', door_value)

    except requests.exceptions.RequestException as e:
        print('HTTP Request failed:', e)
    
    return door_value

# Ask the user for the IP address
ip_address = input("Enter the IP address of the server: ")

# Call the function with the user-provided IP address
door_value = make_http_post_request(ip_address)

# Print the result outside the function
print('The action was applied to:', door_value)
