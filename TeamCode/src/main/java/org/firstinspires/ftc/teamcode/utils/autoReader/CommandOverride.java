package org.firstinspires.ftc.teamcode.utils.autoReader;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)
public class CommandOverride {
    public String cmdId;
    public boolean skip;
    public String pathId;
}
