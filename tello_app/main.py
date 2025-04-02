import subprocess
from flask import Flask, jsonify, request, render_template

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/get_wifi_networks")
def get_wifi_networks():
    wifis = subprocess.check_output("nmcli -f SSID,SECURITY device wifi list", shell=True, text=True)
    processed_wifis = [net.split() for net in wifis.split("\n")[1:-1]]
    wifi_set = set(w[0] for w in processed_wifis)

    networks = []

    for wifi in processed_wifis:
        if wifi[0] in wifi_set:
            networks.append((wifi[0], False if wifi[1] == "--" else True))
            wifi_set.remove(wifi[0])

    return jsonify({"networks": networks})

@app.route("/get_connected_network")
def get_connected_network():
    wifi = subprocess.check_output("nmcli -t -f active,ssid dev wifi | awk -F: '$1==\"yes\" {print $2}'", shell=True, text=True)
    return jsonify({"network": wifi.split("\n")[0]})

@app.route("/connect_wifi", methods=["POST"])
def connect_wifi():
    if request.method == "POST":
        network = request.form.get("network")
        pw = request.form.get("password")
        command = f'nmcli device wifi connect {network}'
        if pw:
            command += f' password {pw}'

if __name__ == "__main__":
    app.run(debug=True)