## store some helper function methods used to calculate bit masking or other useful things here.

def calcMaskFromCheckboxes(checkBoxList):
    s = [ 1<<i if checkBoxList[i].isChecked() else 0 for i in range(len(checkBoxList))]
    mask = sum(s)
    return mask