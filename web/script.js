// BLE communication and data handling

let bleCharacteristic = null;

async function connectToBLE() {
    try {
        const device = await navigator.bluetooth.requestDevice({
            acceptAllDevices: true,
            optionalServices: ['12345678-1234-5678-1234-56789abcdef0']
        });
        const server = await device.gatt.connect();
        const service = await server.getPrimaryService('12345678-1234-5678-1234-56789abcdef0');
        bleCharacteristic = await service.getCharacteristic('abcdef01-1234-5678-1234-56789abcdef0');
        console.log("Connected to BLE device.");

        // Add BLE notification handler to update the web page
        bleCharacteristic.addEventListener('characteristicvaluechanged', (event) => {
            const decoder = new TextDecoder();
            const value = decoder.decode(event.target.value);
            console.log("Received BLE notification:", value);

            if (value.startsWith("FUEL:")) {
                const [_, fuel, burnRate, time] = value.split(':');
                updateFuelDisplay(parseFloat(fuel), parseFloat(burnRate), parseFloat(time));
            } else if (value.startsWith("Updated POIs:")) {
                console.log("POI update confirmation received.");
            }
        });
    } catch (error) {
        console.error("Failed to connect to BLE device:", error);
    }
}

async function sendFuelSettings(fuel, burnRate, visible) {
    const data = `SET_FUEL:${fuel}:${burnRate}:${visible ? 1 : 0}`;
    await sendBLEData(data);
    console.log("Fuel settings sent:", data);
}

async function sendPOISettings(pois) {
    let data = "SET_POI:";
    pois.forEach((poi, index) => {
        data += `POI${index + 1}:${poi.name}:${poi.lat}:${poi.lon},`;
    });
    await sendBLEData(data);
    console.log("POI settings sent:", data);
}

async function requestFuelData() {
    await sendBLEData("GET_FUEL");
    console.log("Requested fuel data.");
}

async function sendBLEData(data) {
    if (bleCharacteristic) {
        const encoder = new TextEncoder();
        await bleCharacteristic.writeValue(encoder.encode(data));
    } else {
        console.error("BLE characteristic not connected.");
    }
}

function updateFuelDisplay(fuel, burnRate, time) {
    document.getElementById("fuel").innerText = `Fuel: ${fuel} L`;
    document.getElementById("burnRate").innerText = `Burn Rate: ${burnRate} L/h`;
    document.getElementById("time").innerText = `Time: ${time} h`;
}

function updatePOIDisplay(pois) {
    pois.forEach((poi, index) => {
        document.getElementById(`poi${index + 1}`).innerText = `${poi.name} (${poi.lat}, ${poi.lon})`;
    });
}

// Event listeners for buttons
document.getElementById("connectButton").addEventListener("click", connectToBLE);
document.getElementById("requestFuelButton").addEventListener("click", requestFuelData);
document.getElementById("sendFuelButton").addEventListener("click", () => {
    const fuel = parseFloat(document.getElementById("fuelInput").value);
    const burnRate = parseFloat(document.getElementById("burnRateInput").value);
    const visible = document.getElementById("fuelVisibleInput").checked;
    sendFuelSettings(fuel, burnRate, visible);
});
document.getElementById("sendPOIButton").addEventListener("click", () => {
    const pois = [
        {
            name: document.getElementById("poi1Name").value,
            lat: parseFloat(document.getElementById("poi1Lat").value),
            lon: parseFloat(document.getElementById("poi1Lon").value)
        },
        {
            name: document.getElementById("poi2Name").value,
            lat: parseFloat(document.getElementById("poi2Lat").value),
            lon: parseFloat(document.getElementById("poi2Lon").value)
        },
        {
            name: document.getElementById("poi3Name").value,
            lat: parseFloat(document.getElementById("poi3Lat").value),
            lon: parseFloat(document.getElementById("poi3Lon").value)
        }
    ];
    sendPOISettings(pois);
});