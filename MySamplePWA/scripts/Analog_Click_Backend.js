document.addEventListener('click', parseScreenClick, false);
// const canvas1 = document.getElementById("canvas");

var store_rect_interval;
const button_colour_delay = 150;

function parseScreenClick(event){
    const canvas = document.getElementById("canvas");
    if (canvas.getContext) {
        const ctx = canvas.getContext("2d");
        var x = event.offsetX;
        var y = event.offsetY;
        // console.log("Clicked " + x + " " + y);

        // Start Analog Screen Click Parsing
        if(analog_stick_flag && ctx.isPointInPath(analog_stick_bounding_rect, x, y)) {
            // Do Something with the click
            console.log("Clicked the analog stick");
            onAnalogStickClick();
        }
        if(c_stick_flag && ctx.isPointInPath(c_stick_bounding_rect, x, y)) {
            // Do Something with the click
            console.log("Clicked the c-stick");
            onCStickClick();
        }
        if(store_val_flag && ctx.isPointInPath(store_value_rect, x, y)){
            store_val_colour_flag = 1;
            setTimeout(() => {
                store_val_colour_flag = 0;
              }, button_colour_delay);

            redo_last_store_flag = 1;
            storeValue();
        }
        if(redo_last_store_flag && ctx.isPointInPath(redo_last_rect, x, y)){
            redo_last_store_colour_flag = 1;
            setTimeout(() => {
                redo_last_store_colour_flag = 0;
              }, button_colour_delay);

            redoStorageButtonClicked();
        }
        if(done_calib_flag && ctx.isPointInPath(done_calib_rect, x, y)){
            done_calib_colour_flag = 1;
            setTimeout(() => {
                done_calib_colour_flag = 0;
              }, button_colour_delay);

            doneCalibration();
        }
        if(get_current_cal_flag && ctx.isPointInPath(get_current_cal_rect, x, y)){
            get_current_cal_colour_flag = 1;
            setTimeout(() => {
                get_current_cal_colour_flag = 0;
              }, button_colour_delay);

            requestAnalogCalibration();
            console.log("request analog calibration values");
        }
        if(send_calib_flag && ctx.isPointInPath(send_calib_rect, x, y)){
            send_calib_colour_flag = 1;
            setTimeout(() => {
                send_calib_colour_flag = 0;
              }, button_colour_delay);

            sendStickCalibration();
            setTimeout(() => {
                sendStickDeadzones();
              }, 1000);
        }
        if(save_calib_flag && ctx.isPointInPath(save_calib_rect, x, y)){
            save_calib_colour_flag = 1;
            setTimeout(() => {
                save_calib_colour_flag = 0;
              }, button_colour_delay);

            saveCalibValues();
        }
        if(deadzones_flag && ctx.isPointInPath(deadzones_rect, x, y)){
            deadzones_colour_flag = 1;
            setTimeout(() => {
                deadzones_colour_flag = 0;
              }, button_colour_delay);

            // sendMSG("RDC");
            setTimeout(() => {
                // editDeadzones();
                showDeadzonePopup();
            }, 1000);
        }
        if(toggle_triggers_flag && ready_toggle == 0 && ctx.isPointInPath(toggle_L_trigger_rect,x,y)){
            toggle_L_trigger_colour_flag = 1;
            setTimeout(() => {
                toggle_L_trigger_colour_flag = 0;
              }, button_colour_delay);
            setTimeout(() => {
                sendMSG("A");
              }, 1000);
            if(L_Trigger_on){
                L_Trigger_on = 0;
            }
            else{
                L_Trigger_on = 1;
            }
            var trig_msg = "T" + L_Trigger_on.toString() + R_Trigger_on.toString();
            sendMSG(trig_msg);
        }
        if(toggle_triggers_flag && ready_toggle == 0 && ctx.isPointInPath(toggle_R_trigger_rect,x,y)){
            toggle_R_trigger_colour_flag = 1;
            setTimeout(() => {
                toggle_R_trigger_colour_flag = 0;
              }, button_colour_delay);
            setTimeout(() => {
                sendMSG("A");
            }, 1000);
            if(R_Trigger_on){
                R_Trigger_on = 0;
            }
            else{
                R_Trigger_on = 1;
            }
            var trig_msg = "T" + L_Trigger_on.toString() + R_Trigger_on.toString();
            sendMSG(trig_msg);
        }
        if(toggle_stick_raw_flag && ctx.isPointInPath(toggle_stick_raw_rect,x,y)){
            toggle_stick_raw_colour_flag = 1;
            setTimeout(() => {
                toggle_stick_raw_colour_flag = 0;
              }, button_colour_delay);
            if(stick_raw_on_flag){
                stick_raw_on_flag = 0;
            }
            else{
                stick_raw_on_flag = 1;
            }
        }
        if(trigger_flag && ctx.isPointInPath(left_trig_base_path, x, y)){
            console.log("Left trigger clicked");
        }
        if(trigger_flag && ctx.isPointInPath(right_trig_base_path, x, y)){
            console.log("Rigth trigger clicked");
        }
        // if(deadzones_flag && ctx.isPointInPath(send_deadzones_rect, x, y)){
        //     sendStickDeadzones();
        // }
        // if(deadzones_flag && ctx.isPointInPath(save_deadzones_rect, x, y)){
        //     saveStickDeadzones();
        // }
        if(finished_calib_flag && ctx.isPointInPath(finished_calib_rect, x, y)){
            finished_calib_colour_flag = 1;
            setTimeout(() => {
                finished_calib_colour_flag = 0;
              }, button_colour_delay);

            finishedCalibration();
        }
        // End Analog Screen Click Parsing

        //Start Digital Screen Click Parsing
        if(A_Button_flag && ctx.isPointInPath(A_Button_Path,x,y)){
            console.log("Clicked A Button");
            onGCSimButtonClicked("A");
            A_Colour_flag = 1;
            setTimeout(() => {
                A_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(B_Button_flag && ctx.isPointInPath(B_Button_Path,x,y)){
            console.log("Clicked B Button");
            onGCSimButtonClicked("B");
            B_Colour_flag = 1;
            setTimeout(() => {
                B_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(S_Button_flag && ctx.isPointInPath(S_Button_Path,x,y)){
            console.log("Clicked Start Button");
            onGCSimButtonClicked("S");
            S_Colour_flag = 1;
            setTimeout(() => {
                S_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(D_Button_flag && ctx.isPointInPath(DU_Button_Path,x,y)){
            console.log("Clicked D-Pad Up Button");
            onGCSimButtonClicked("u");
            u_Colour_flag = 1;
            setTimeout(() => {
                u_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(D_Button_flag && ctx.isPointInPath(DR_Button_Path,x,y)){
            console.log("Clicked D-Pad Rigth Button");
            onGCSimButtonClicked("r");
            r_Colour_flag = 1;
            setTimeout(() => {
                r_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(D_Button_flag && ctx.isPointInPath(DD_Button_Path,x,y)){
            console.log("Clicked D-Pad Down Button");
            onGCSimButtonClicked("d");
            d_Colour_flag = 1;
            setTimeout(() => {
                d_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(D_Button_flag && ctx.isPointInPath(DL_Button_Path,x,y)){
            console.log("Clicked D-Pad Left Button");
            onGCSimButtonClicked("l");
            l_Colour_flag = 1;
            setTimeout(() => {
                l_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(X_Button_flag && ctx.isPointInPath(X_Button_Path,x,y)){
            console.log("Clicked X Button");
            onGCSimButtonClicked("X");
            X_Colour_flag = 1;
            setTimeout(() => {
                X_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(Y_Button_flag && ctx.isPointInPath(Y_Button_Path,x,y)){
            console.log("Clicked Y Button");
            onGCSimButtonClicked("Y");
            Y_Colour_flag = 1;
            setTimeout(() => {
                Y_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(Z_Button_flag && ctx.isPointInPath(Z_Button_Path,x,y)){
            console.log("Clicked Z Button");
            onGCSimButtonClicked("Z");
            Z_Colour_flag = 1;
            setTimeout(() => {
                Z_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(L_Button_flag && ctx.isPointInPath(L_Button_Path,x,y)){
            console.log("Clicked L Button");
            onGCSimButtonClicked("L");
            L_Colour_flag = 1;
            setTimeout(() => {
                L_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(R_Button_flag && ctx.isPointInPath(R_Button_Path,x,y)){
            console.log("Clicked R Button");
            onGCSimButtonClicked("R");
            R_Colour_flag = 1;
            setTimeout(() => {
                R_Colour_flag = 0;
            }, button_colour_delay);
        }
        if(Read_Mapping_Button_Flag && ctx.isPointInPath(Read_Mapping_Button_Path,x,y)){
            Read_Mapping_Button_Colour_Flag = 1;
            setTimeout(() => {
                Read_Mapping_Button_Colour_Flag = 0;
              }, button_colour_delay);

            console.log("Read Mapping Button Clicked");
            requestButtonMapping();
        }
        if(Send_Mapping_Button_Flag && ctx.isPointInPath(Send_Mapping_Button_Path,x,y)){
            Send_Mapping_Button_Colour_Flag = 1;
            setTimeout(() => {
                Send_Mapping_Button_Colour_Flag = 0;
              }, button_colour_delay);

            console.log("Send Mapping Button Clicked");
            sendButtonMapping();
        }
        if(Save_Mapping_Button_Flag && ctx.isPointInPath(Save_Mapping_Button_Path,x,y)){
            Save_Mapping_Button_Colour_Flag = 1;
            setTimeout(() => {
                Save_Mapping_Button_Colour_Flag = 0;
              }, button_colour_delay);

            saveButtonMapping();
            console.log("Save Mapping Button Clicked");
        }
        if(Redo_Last_Button_Flag && ctx.isPointInPath(Redo_Last_Button_Path,x,y)){
            Redo_Last_Button_Colour_Flag = 1;
            setTimeout(() => {
                Redo_Last_Button_Colour_Flag = 0;
              }, button_colour_delay);

            console.log("Redo Last Button clicked");
            redoLastButtonClick();
        }
        if(Reset_Default_Mapping_Flag && ctx.isPointInPath(Reset_Default_Mapping_Path,x,y)){
            Reset_Default_Mapping_Colour_Flag = 1;
            setTimeout(() => {
                Reset_Default_Mapping_Colour_Flag = 0;
              }, button_colour_delay);

            console.log("Reset Mapping Button clicked");
            setDefaultMapping();
        }
        if(Done_Mapping_Button_Flag && ctx.isPointInPath(Done_Mapping_Button_Path,x,y)){
            Done_Mapping_Button_Colour_Flag = 1;
            setTimeout(() => {
                Done_Mapping_Button_Colour_Flag = 0;
              }, button_colour_delay);

            finishedDigitalSettings();
            console.log("Done Mapping Button Clicked");
        }
        //End Digital Screen Click Parsing

    }
}

function onAnalogStickClick(){
    store_val_flag = 1;
    done_calib_flag = 1;
    deadzones_flag = 1;
    toggle_triggers_flag = 0;
    toggle_stick_raw_flag = 0;
    storageCounter = 0;
}

function onCStickClick(){
    skipToCStick();
    store_val_flag = 1;
    done_calib_flag = 1;
    deadzones_flag = 1;
    toggle_triggers_flag = 0;
    toggle_stick_raw_flag = 0;
}

function onGCSimButtonClicked(tag){
    switch (num_clicked){
        case 0:
            first_clicked = tag;
            num_clicked++;
            break;
        case 1:
            second_clicked = tag;
            num_clicked = 0;
            twoClicked();
            break;
    }
    Redo_Last_Button_Flag = num_clicked;
}
