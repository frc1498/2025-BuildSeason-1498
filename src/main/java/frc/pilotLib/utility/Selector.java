package frc.pilotLib.utility;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Selector extends SubsystemBase{
    private int currentSelection;
    private String currentSelectionName;
    private ArrayList<String> selections;
    private String filterCriteria = "";

    public Selector() {
        //Empty constructor.
        currentSelection = 0;
        currentSelectionName = "";
        selections = new ArrayList<String>();
    }

    public Selector(ArrayList<String> selections) {
        this();
        this.selections = this.addSelectionList(selections);
        this.sortSelections();
    }

    public Selector(ArrayList<String> selections, String name) {
        this();
        this.selections = this.addSelectionList(selections);
        this.sortSelections();
        this.setSmartDashboardName(name);
    }

    public Selector(File folderPath) {
        this();
        this.selections = addSelectionList(folderPath);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension) {
        this();
        this.selections = this.addSelectionList(folderPath, extension);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension, String name) {
        this();
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

    private ArrayList<String> getSelectionList() {
        return this.selections;
    }

    private String getCurrentSelectionName() {
        return this.currentSelectionName;
    }

    private void setCurrentSelectionName() {
        this.currentSelectionName = this.selections.get(this.currentSelection);
    }

    private int getCurrentIndex() {
        return this.currentSelection;
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
        else {
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
        else {
            this.currentSelection = 0;
        }
    }

    private ArrayList<String> filterSelections(ArrayList<String> list, Supplier<String> filterCriteria) {
        Iterator<String> filter = list.iterator();
        ArrayList<String> placeholder = new ArrayList<String>();
        String toCheck;

        this.filterCriteria = filterCriteria.get();

        while(filter.hasNext()) {
            toCheck = filter.next();
            if (toCheck.contains(filterCriteria.get())) {
                placeholder.add(toCheck);
            }
        }
        return placeholder;
    }

    private void sortSelections() {
         this.selections.sort(Comparator.naturalOrder());
         this.setCurrentSelectionName();
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    //For debugging only.
    private String[] getCurrentListArray() {
        return this.getSelectionList().toArray(new String[this.getSelectionList().size()]);
    }

    public Command filterList(Supplier<String> criteria) {
        return runOnce(
            () -> {
                this.selections = filterSelections(this.selections, criteria);
                this.sortSelections();
                this.setSelection(0);
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("filterList");
    }

    public Command increment() {
        return runOnce(
            () -> {
                this.incrementSelection();
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("increment");
    }

    public Command decrement() {
        return runOnce(
            () -> {
                this.decrementSelection();
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("decrement");
    }

    public Supplier<ArrayList<String>> currentList() {
        return this::getSelectionList;
    }

    public Supplier<Integer> currentIndex() {
        return this::getCurrentIndex;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Current Command", this::getCurrentCommandName, null);
        builder.addStringProperty("Current Selection", this::getCurrentSelectionName, null);
        builder.addStringProperty("Filter String", () -> {return this.filterCriteria;}, null);
        builder.addIntegerProperty("Number of Selections", () -> {return this.selections.size();}, null);
        builder.addStringArrayProperty("Current List", this::getCurrentListArray, null);
    }
}
