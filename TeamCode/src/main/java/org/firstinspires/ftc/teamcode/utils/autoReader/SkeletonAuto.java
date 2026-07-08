package org.firstinspires.ftc.teamcode.utils.autoReader;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class SkeletonAuto {
    public String name;
    public List<SkeletonCommand> commands;
}
