package org.firstinspires.ftc.teamcode.utils.autoReader;

import com.acmerobotics.roadrunner.Action;

/**
 * Fluent builder for BrainstemPilot autos and standalone paths. By default, builds for the side
 * marked in the path's {@code startSide} field; use {@link #forSide(FieldSide)} or
 * {@link #mirrorSide()} to run on the opposite side.
 */
public class PilotAutoBuilder {

    enum Target {
        AUTO,
        PATH
    }

    private final String m_name;
    private final Target m_target;
    private FieldSide m_runSide = null;

    PilotAutoBuilder(String name, Target target) {
        m_name = name;
        m_target = target;
    }

    static PilotAutoBuilder forAuto(String variantAutoName) {
        return new PilotAutoBuilder(variantAutoName, Target.AUTO);
    }

    static PilotAutoBuilder forPath(String pathId) {
        return new PilotAutoBuilder(pathId, Target.PATH);
    }

    public PilotAutoBuilder forSide(FieldSide side) {
        m_runSide = side;
        return this;
    }

    public PilotAutoBuilder mirrorSide() {
        m_runSide = getAuthoredStartSide().opposite();
        return this;
    }

    public Action build() {
        FieldSide runSide = m_runSide != null ? m_runSide : getAuthoredStartSide();
        return m_target == Target.AUTO
                ? BrainstemPilot.buildAutoInternal(m_name, runSide)
                : BrainstemPilot.buildPathInternal(m_name, runSide);
    }

    private FieldSide getAuthoredStartSide() {
        return m_target == Target.AUTO
                ? BrainstemPilot.getAuthoredStartSide(m_name)
                : BrainstemPilot.getPathStartSide(m_name);
    }
}
