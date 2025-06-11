import json
import requests

ip = '192.168.69.1'

data = {
	'blueVlans': '40_50_60',
	'channel': 13,
	'channelBandwidth': '40MHz',
	'redVlans': '10_20_30',
	'stationConfigurations': {
		'blue1': {
			'ssid': '3061-Calypso',
			'wpaKey': '30613061'
		},
		'blue2': {
			'ssid': '9999-Prac',
			'wpaKey': '30613061'
		},
		'blue3': {
			'ssid': '',
			'wpaKey': ''
		},
		'red1': {
			'ssid': '',
			'wpaKey': ''
		},
		'red2': {
			'ssid': '',
			'wpaKey': ''
		},
		'red3': {
			'ssid': '',
			'wpaKey': ''
		}
	}
}

r = requests.post(f'http://{ip}/configuration', json.dumps(data))

print(r.status_code, r.reason)