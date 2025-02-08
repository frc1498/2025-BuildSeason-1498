package frc.pilotLib.utility;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Selector {
    private int currentSelection;
    private String currentSelectionName;
    ArrayList<String> selections;

    public Selector() {
        //Empty constructor.
    }

    private void addSelectionList(ArrayList<String> selectionList) {
        this.selections = selectionList;
    }

    private void addSelectionList(File folderPath) {
        for (var i : folderPath.listFiles()) {
            if(i.isFile()) {
                this.selections.add(i.getName());
            }
        }
    }

    private void removeSubstringFromList(ArrayList<String> list, String substring) {
        for (var i: list) {
            
        }
    }

    private void addSelectionList(File folderPath, String extension) {
        
    }
}
