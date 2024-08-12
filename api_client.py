import requests
from robot.api.deco import keyword

TOKEN = None

@keyword("Authenticate")
def authenticate_token(token_url, username, password):
    global TOKEN
    data = {
        'username': username,
        'password': password
    }
    response = requests.post(token_url, data=data)
    response.raise_for_status()  # Check if the request was successful
    TOKEN = response.json().get("access_token")
    return TOKEN

@keyword("Get Token")
def get_headers():
    global TOKEN
    if not TOKEN:
        raise Exception("No token found, please authenticate first.")
    return {
        'Authorization': f'Bearer {TOKEN}'
    }

@keyword("GetRequest")
def get_endpoint(base_url, endpoint=None):
    if not endpoint:
        url = base_url
    else:
        url = f"{base_url}/{endpoint}"
    headers = get_headers()
    try:
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"An error occurred when accessing the protected endpoint: {e}")

@keyword("Post")
def post_to_endpoint(base_url, endpoint=None, data=None):
    if not endpoint:
        url = base_url
    else:
        url = f"{base_url}/{endpoint}"
    headers = get_headers()
    try:
        response = requests.post(url, headers=headers, json=data)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"An error occurred {endpoint}: {e}")

    
