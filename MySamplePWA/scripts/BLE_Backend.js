var BLE_Server;
var currently_sending;
var PASSWORD_CH;
var password_correct = 0;
var in_window_index = 0;
var connected_flag = 0;
var reset_password = 0;


function isWebBluetoothEnabled() {
    if(connected_flag == 0){
        // this is what generates the BLE pop up searching window
        navigator.bluetooth.requestDevice({
            filters: [{
                services: ['4fafc201-1fb5-459e-8fcc-c5c9c331914b']
            }],
            optionalServices: ["4fafc201-1fb5-459e-8fcc-c5c9c331914b"]
        })

        .then(device => {
            device.addEventListener( 'gattserverdisconnected', handleDisconnect );
            return device.gatt.connect();
        })
        .then(server => {
            BLE_Server = server;
            console.log("Connected");
            return server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
        })
        .then(service => {
            return service.getCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
        })
        .then(characteristic => {
            if (characteristic.properties.notify){
                PASSWORD_CH = characteristic;
                characteristic.addEventListener("characteristicvaluechanged",handleNewPassMSG);
                characteristic.startNotifications();
                console.log("listening for incoming data");
            }
            return 0;
        })
        .then(server => { /* … */ })
        .catch(error => { console.log(error); });
    }
    else{
        showBLENameChangePopup();
    }
}


function sendMSG(msg){
    console.log("Sending : " + msg);
    if(connected_flag){
        // console.log("Sending : " + msg);
        currently_sending = msg;
        BLE_Server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
        .then(service => {
            return service.getCharacteristic("da1e7d98-916b-11ed-a1eb-0242ac120002");
        })
        .then(characteristic => {
            var enc = new TextEncoder();
            characteristic.writeValue(enc.encode(msg));
        })
        .catch(error => { console.error(error); });
    }
}


async function handleNewPassMSG(event){
    connected_flag = 1;
    const value = event.target.value;
    var enc = new TextDecoder("utf-8");
    var reading = enc.decode(value);
    if(reading.startsWith("Connected to:") || reading.startsWith("Password Correct")){ // for backwards compatability with v1.1.1 alpha and below
        console.log("Password Correct");
        document.getElementById("on screen information").innerHTML = "Password Correct";
        document.getElementById("Firmware Version").innerHTML = reading;
        PASSWORD_CH.stopNotifications();
        PASSWORD_CH.removeEventListener("characteristicvaluechanged",handleNewPassMSG);
        password_correct = 1;
        changeButtonNames();
    }
    else if(reading == "Reset Password" && reset_password == 0){
        document.getElementById("on screen information").innerHTML = "Reset Password";
        reset_password = 1;
    }
    else if(reading == "Password Reset"){
        reset_password = 0;
    }
    else{
        console.log(reading);
        document.getElementById("on screen information").innerHTML = reading;
    }
}


function showBLENameChangePopup(){
    document.getElementById("ble_popup").style.display = "block";
    document.getElementById("ble_popup").addEventListener("keyup", handleEnterBLE);
}

function handleEnterBLE(event){
    event.preventDefault();
    if (event.key === "Enter") {
        doneNewBLEName();
    }
}


function doneNewBLEName(){
    const new_ble_name = document.getElementById("newBLEName").value;

    if(new_ble_name.length>=5 && new_ble_name.length<=14){
        document.getElementById("newBLEName").value = "";
        document.getElementById("ble_popup").style.display = "none";
        document.getElementById("ble_popup").removeEventListener("keyup",handleEnterBLE);
        
        const msg = "B" + new_ble_name;
        sendMSG(msg);
        setTimeout(() => {
            sendMSG("Standby");
        }, 1000);

    }
    else{
        document.getElementById("BLEChangeMsg").innerHTML = "Outside [5,14] character range";
    }
}  



function closeNewBLEName(){
    document.getElementById("newBLEName").value = "";
    document.getElementById("ble_popup").style.display = "none";
    document.getElementById("ble_popup").removeEventListener("keyup",handleEnterBLE);
}

function changeButtonNames(){
    if(connected_flag){
        document.getElementById("Connect_Button").style.fontSize = "20px";
        document.getElementById("Password_Button").style.fontSize = "20px";
        document.getElementById("Connect_Button").innerHTML = "Change BLE Name";
        document.getElementById("Password_Button").innerHTML = "Change Password";
    }
    else{
        document.getElementById("Connect_Button").style.fontSize = "28px";
        document.getElementById("Password_Button").style.fontSize = "28px";
        document.getElementById("Connect_Button").innerHTML = "Connect";
        document.getElementById("Password_Button").innerHTML = "Password";
    }
}


function handleDisconnect(event){
    console.log(`${event.target.name} is disconnected`);
    document.getElementById("on screen information").innerHTML = `${event.target.name} is disconnected`;
    document.getElementById("Firmware Version").innerHTML = "";
    connected_flag = 0;
    password_correct = 0;
    changeButtonNames();
  };

