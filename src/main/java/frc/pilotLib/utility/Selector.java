package frc.pilotLib.utility;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Selector extends SubsystemBase{
    private int currentSelection;
    private String currentSelectionName;
    private ArrayList<String> selections;

    public Selector() {
        //Empty constructor.
        currentSelection = 0;
        currentSelectionName = "";
        selections = new ArrayList<String>();
    }

    public Selector(ArrayList<String> selections) {
        super();
        this.selections = this.addSelectionList(selections);
        this.sortSelections();
    }

    public Selector(ArrayList<String> selections, String name) {
        super();
        this.selections = this.addSelectionList(selections);
        this.sortSelections();
        this.setSmartDashboardName(name);
    }

    public Selector(File folderPath) {
        super();
        this.selections = addSelectionList(folderPath);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension) {
        super();
        this.selections = this.addSelectionList(folderPath, extension);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension, String name) {
        super();
        this.selections = this.addSelectionList(folderPath, extension);
        this.sortSelections();
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

    private void setSelection(int index) {
        if (index < 0) {
            this.currentSelection = 0;
        }
        else if (index > this.selections.size() - 1) {
            this.currentSelection = this.selections.size();
        }
        else {
            this.currentSelection = index;
        }
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

    private ArrayList<String> filterSelections(ArrayList<String> list, String filterCriteria) {
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

    private void sortSelections() {
         this.selections.sort(Comparator.naturalOrder());
    }

    public Command filterList(String criteria) {
        return runOnce(
            () -> {
                this.selections = filterSelections(this.selections, criteria);
                this.sortSelections();
                this.setSelection(0);
                this.setCurrentSelectionName();
            }
        ).withName("filterList");
    }

    public Command increment() {
        return runOnce(
            () -> {
                this.incrementSelection();
                this.setCurrentSelectionName();
            }
        ).withName("increment");
    }

    public Command decrement() {
        return runOnce(
            () -> {
                this.decrementSelection();
                this.setCurrentSelectionName();
            }
        ).withName("decrement");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Current Selection", this::getCurrentSelectionName, null);
    }
}
