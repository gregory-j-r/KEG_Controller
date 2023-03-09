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

            sendMSG("RDC");
            setTimeout(() => {
                editDeadzones();
            }, 1000);
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
        }
        if(B_Button_flag && ctx.isPointInPath(B_Button_Path,x,y)){
            console.log("Clicked B Button");
            onGCSimButtonClicked("B");
        }
        if(S_Button_flag && ctx.isPointInPath(S_Button_Path,x,y)){
            console.log("Clicked Start Button");
            onGCSimButtonClicked("S");
        }
        if(D_Button_flag && ctx.isPointInPath(DU_Button_Path,x,y)){
            console.log("Clicked D-Pad Up Button");
            onGCSimButtonClicked("u");
        }
        if(D_Button_flag && ctx.isPointInPath(DR_Button_Path,x,y)){
            console.log("Clicked D-Pad Rigth Button");
            onGCSimButtonClicked("r");
        }
        if(D_Button_flag && ctx.isPointInPath(DD_Button_Path,x,y)){
            console.log("Clicked D-Pad Down Button");
            onGCSimButtonClicked("d");
        }
        if(D_Button_flag && ctx.isPointInPath(DL_Button_Path,x,y)){
            console.log("Clicked D-Pad Left Button");
            onGCSimButtonClicked("l");
        }
        if(X_Button_flag && ctx.isPointInPath(X_Button_Path,x,y)){
            console.log("Clicked X Button");
            onGCSimButtonClicked("X");
        }
        if(Y_Button_flag && ctx.isPointInPath(Y_Button_Path,x,y)){
            console.log("Clicked Y Button");
            onGCSimButtonClicked("Y");
        }
        if(Z_Button_flag && ctx.isPointInPath(Z_Button_Path,x,y)){
            console.log("Clicked Z Button");
            onGCSimButtonClicked("Z");
        }
        if(L_Button_flag && ctx.isPointInPath(L_Button_Path,x,y)){
            console.log("Clicked L Button");
            onGCSimButtonClicked("L");
        }
        if(R_Button_flag && ctx.isPointInPath(R_Button_Path,x,y)){
            console.log("Clicked R Button");
            onGCSimButtonClicked("R");
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
    storageCounter = 0;
}

function onCStickClick(){
    skipToCStick();
    store_val_flag = 1;
    done_calib_flag = 1;
    deadzones_flag = 1;
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
