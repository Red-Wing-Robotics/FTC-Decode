package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

abstract public class RWRBaseOpMode extends OpMode {
    public final Logger logger;

    public RWRBaseOpMode () {
        super();
        this.logger = new Logger(this.telemetry);
    }

}
