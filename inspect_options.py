
import adsk.core, adsk.fusion, traceback

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        exportMgr = design.exportManager
        
        # Create a dummy options object (needs an input component)
        root = design.rootComponent
        if root.allOccurrences.count > 0:
            occ = root.allOccurrences.item(0)
            objOptions = exportMgr.createOBJExportOptions(root, "test")
            
            props = dir(objOptions)
            ui.messageBox(f"OBJ Options: {props}")
            
            # Check for generic 'units' or similar
            # And stl
            stlOptions = exportMgr.createSTLExportOptions(root, "test")
            ui.messageBox(f"STL Options: {dir(stlOptions)}")

        else:
            ui.messageBox("No components to test with.")

    except:
        ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
