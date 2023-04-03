
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
            if(i%4 == 0){
                InGameReplyAsBits += "\n";
            }
            val = InGameReplyArr[i]
            InGameReplyAsBits += To2Bits(val)
        }
        console.log("In Game Reply = " +InGameReplyAsBits);
        console.log()
    } 
    else if(debug_input == "2"){
        var bufferHolderAsBits = ""
        for(let i = 0; i<=str5.length-3; i+=3){
            if(i%4 == 0){
                bufferHolderAsBits += "\n";
            }
            bufferHolderAsBits += To2Bits(str5.substring(i,i+3));
        }
        console.log(bufferHolderAsBits);
        // console.log(str5)
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

function To2Bits(val){
    var ret_val = val;
    if(val == "004"){
        ret_val =  "00";
    }
    else if(val == "055"){
        ret_val = "11";
    }
    else if(val == "052"){
        ret_val = "01";
    }
    else if(val == "007"){
        ret_val = "10";
    }
    else if(val == "063"){
        ret_val = "STOP";
    }
    return ret_val;
}