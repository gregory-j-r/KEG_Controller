


function getPassword(){
    if(connected_flag){
        if(password_correct || reset_password){
            console.log("password correct = " + password_correct);
            console.log("reset password = " + reset_password);
            showPassResetPopup();
        }
        else{
            const pass = window.prompt("What's your KEG Controller Password?\nPlease Hold X+Y while you submit your password");
            if(pass!=null){
                sendMSG(pass);
                setTimeout(() => {
                    sendMSG("Standby");
                  }, 1000);
            }
        }
    }
    else{
        console.log("Connect to controller first");
        document.getElementById("on screen information").innerHTML = "Connect to controller first";
    }
}




function showPassResetPopup() {
    document.getElementById("password_reset_popup").style.display = "block";
    document.getElementById("password_reset_popup").addEventListener("keyup", handleEnterChangePass);
}

function handleEnterChangePass(event){
    event.preventDefault();
    if (event.key === "Enter") {
        doneNewPassword();
    }
}

function toggleHideNewPass(){
    const new_pass_input = document.getElementById("NewPass");
    const re_new_pass_input = document.getElementById("ReNewPass");
    if (new_pass_input.getAttribute("type") === "password") {
        new_pass_input.setAttribute("type", "text");
    } else {
        new_pass_input.setAttribute("type", "password");
    }

    if (re_new_pass_input.getAttribute("type") === "password") {
        re_new_pass_input.setAttribute("type", "text");
    } else {
        re_new_pass_input.setAttribute("type", "password");
    }
}

function doneNewPassword(){
    const new_pass = document.getElementById("NewPass").value;
    const re_new_pass = document.getElementById("ReNewPass").value;

    if(new_pass == re_new_pass){
        if(new_pass.length>=5 && new_pass.length<15){
            document.getElementById("NewPass").value = "";
            document.getElementById("ReNewPass").value = "";
            document.getElementById("password_reset_popup").style.display = "none";
            document.getElementById("password_reset_popup").removeEventListener("keyup",handleEnterChangePass);
            
            const msg = "P" + new_pass;
            sendMSG(msg);
            setTimeout(() => {
                sendMSG("Standby");
            }, 1000);
    
        }
        else{
            document.getElementById("passChangeMsg").innerHTML = "Outside [5,14] character range";
        }
    }
    else{
        document.getElementById("passChangeMsg").innerHTML = "Entries do not match";
    }   
}

function closeNewPass(){
    document.getElementById("NewPass").value = "";
    document.getElementById("ReNewPass").value = "";
    document.getElementById("password_reset_popup").style.display = "none";
    document.getElementById("password_reset_popup").removeEventListener("keyup",handleEnterChangePass);
}