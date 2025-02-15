package frc.pilotLib.utility;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Selector implements Sendable{
    private int currentSelection;
    private String currentSelectionName;
    ArrayList<String> selections;

    public Selector() {
        //Empty constructor.
        currentSelection = 0;
        currentSelectionName = "";
        selections = new ArrayList<String>();
    }


    public Selector(ArrayList<String> selections) {
        super();
        this.selections = this.addSelectionList(selections);
    }

    public Selector(ArrayList<String> selections, String name) {
        super();
        this.selections = this.addSelectionList(selections);
        this.setSmartDashboardName(name);
    }

    public Selector(File folderPath) {
        super();
        this.selections = addSelectionList(folderPath);
    }

    public Selector(File folderPath, String extension) {
        super();
        this.selections = this.addSelectionList(folderPath, extension);
    }

    public Selector(File folderPath, String extension, String name) {
        super();
        this.selections = this.addSelectionList(folderPath, extension);
        this.setSmartDashboardName(name);
    }

    private void setSmartDashboardName(String name) {
        SmartDashboard.putData(name, this);
    }

    private ArrayList<String> addSelectionList(ArrayList<String> selectionList) {
        return selectionList;
    }

    private ArrayList<String> addSelectionList(File folderPath) {
        ArrayList<String> placeholder = new ArrayList<String>();
        for (var i : folderPath.listFiles()) {
            if(i.isFile()) {
                placeholder.add(i.getName());
            }
        }
        return placeholder;
    }

    private ArrayList<String> removeSubstringFromList(ArrayList<String> list, String substring) {
        ArrayList<String> placeholder = new ArrayList<String>();
        for (var i: list) {
            placeholder.add(i.replaceFirst(substring, ""));
        }
        return placeholder;
    }

    private ArrayList<String> addSelectionList(File folderPath, String extension) {
        return this.removeSubstringFromList(this.addSelectionList(folderPath), extension);
    }

    private String getCurrentSelectionName() {
        return this.currentSelectionName;
    }

    private void setCurrentSelectionName() {
        this.currentSelectionName = this.selections.get(this.currentSelection);
    }

    private void decrementSelection() {
        if (this.currentSelection > 0) {
            this.currentSelection--;
        }
        else if (this.currentSelection < 0) {
            this.currentSelection = 0;
        }
    }

    private void incrementSelection() {
        if (this.currentSelection < this.selections.size() - 1) {
            this.currentSelection++;
        }
        else if (this.currentSelection > this.selections.size() - 1) {
            this.currentSelection = this.selections.size() - 1;
        }
    }

    public ArrayList<String> filterSelections(ArrayList<String> list, String filterCriteria) {
        Iterator<String> filter = list.iterator();
        ArrayList<String> placeholder = new ArrayList<String>();
        String toCheck;

        while(filter.hasNext()) {
            toCheck = filter.next();
            if (toCheck.contains(filterCriteria)) {
                placeholder.add(toCheck);
            }
        }
        return placeholder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Current Selection", this::getCurrentSelectionName, null);
    }
}
