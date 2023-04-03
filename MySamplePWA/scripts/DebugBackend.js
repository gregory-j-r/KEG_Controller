
var Debug_CH;
var debug_input = ""

function sendDebugMSG(){
    finishedDigitalSettings();
    finishedCalibration();

    debug_input = window.prompt("Which debug message do u want to send?");
    const msg = "Debug" + debug_input;
    sendMSG(msg);

    console.log("Requesting debug data: " + debug_input);
    BLE_Server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    .then(service => {
        return service.getCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
    })
    .then(characteristic => {
        if (characteristic.properties.notify){
            Debug_CH = characteristic;
            characteristic.addEventListener("characteristicvaluechanged",handleNewDebugData);
            characteristic.startNotifications();
        }
        return 0;
    })
    .catch(error => { console.error(error); });
}

function handleNewDebugData(event){
    const value = event.target.value;
    var enc = new TextDecoder("utf-8");
    var str5 = enc.decode(value);
    if(debug_input == "0"){
        console.log("Command byte = " + str5);
    }
    else if(debug_input == "1"){
        var InGameReplyArr = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        var InGameReplyAsBits = ""
        var j = 0;
        for(let i = 0; i<=96; i+=3){
            InGameReplyArr[j] = str5.substring(i,i+3)
            j++;
        }
        var val = ""
        for(let i = 0; i<33; i++){
            val = InGameReplyArr[i]
            if(val == "004"){
                InGameReplyAsBits += "00";
            }
            else if(val == "055"){
                InGameReplyAsBits += "11";
            }
            else if(val == "052"){
                InGameReplyAsBits += "01";
            }
            else if(val == "007"){
                InGameReplyAsBits += "10";
            }
            else if(val == "063"){
                InGameReplyAsBits += "STOP";
            }
            if(i%4 == 0 && i!= 0){
                InGameReplyAsBits += "\n";
            }
        }
        console.log("In Game Reply = " +InGameReplyAsBits);
        console.log()
    }
}

function finishedDebug(){
    try{
        Debug_CH.stopNotifications();
        Debug_CH.removeEventListener("characteristicvaluechanged",handleNewDebugData);
    }
    catch(err){
        console.log(err.message);
    }
}