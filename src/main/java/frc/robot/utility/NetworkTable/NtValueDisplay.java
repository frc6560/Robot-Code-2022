// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.NetworkTable;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utility.AlwaysRunCommand;

/** Add your docs here. */
public class NtValueDisplay<T> {
    final NetworkTable ntTable;
    final NetworkTableEntry entry;
    Supplier<T> value = null;

    public interface DispHelper {
        DispHelper add(String name, Supplier<Object> value);
    }

    public static void ntDisp(String name, Supplier<Object> value){
        ntDisp("dashboard", name, value);
    }

    public static void ntDisp(String tab, String name, Supplier<Object> value){
        NtValueDisplay<Object> valueDisplay = new NtValueDisplay<>(tab, name);

        CommandScheduler.getInstance().schedule(new AlwaysRunCommand(() -> valueDisplay.update(value.get())));
    }

    public static DispHelper ntDispTab(String tab) {
        return new DispHelper(){
            @Override
            public DispHelper add(String name, Supplier<Object> value) {
                ntDisp(tab, name, value);
                return this;
            }
        };
    }


    public NtValueDisplay(String name){
        this("dashboard", name);
    }
    public NtValueDisplay(String tab, String name){
        ntTable = NetworkTableInstance.getDefault().getTable(tab);
        this.entry = ntTable.getEntry(name);
    }
    

    public void update(Object value){
        entry.setValue(value);
    }
}
