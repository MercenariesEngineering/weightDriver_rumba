import urllib.request

print('Downloading Rumba SDK')

url = 'http://guerillarender.com/download/rumba-sdk.zip'
urllib.request.urlretrieve(url, 'rumba-sdk.zip')

print('Unzip Rumba SDK')

import zipfile
with zipfile.ZipFile('rumba-sdk.zip', 'r') as zip_ref:
    zip_ref.extractall(".")
