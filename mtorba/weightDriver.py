from mtorba.exporter import NodeExporter, Input, walk, export_plug
from maya.api.OpenMaya import MFnDagNode

class weightDriverExporter(NodeExporter):

    def __init__(self):
        NodeExporter.__init__(self, "weightDriver", [
            "direction",
            "distanceType",
            "interpolation",
            "kernel",
            "rbfMode",
            "twistAxis",
            "type",
            "active",
            "allowNegativeWeights",
            "angle",
            "bias",
            "centerAngle",
            "driverIndex",
            "evaluate",
            "grow",
            "input",
            "invert",
            "opposite",
            "restInput",
            "scale",
            "translateMax",
            "translateMin",
            "twist",
            "twistAngle",
            "useInterpolation",
            "useRotate",
            "useTranslate",
            "driverMatrix",
            "readerMatrix",
            Input("blendCurve", "blendCurve", "Array",
                  children=["blendCurve_Position", "blendCurve_FloatValue", "blendCurve_Interp"]),
            "poses",
        ], ["output", "outWeight"])

    def walk(self, context, parent, dagNodeFn, node, walk_info):
        NodeExporter.walk(self, context, parent, dagNodeFn, node, walk_info)

        # ** We have to create special connections from driverInput->parentInverseMatrix and driverInput->jointOrient
        # ** The original plug-in do some tricks to not connect those explicitly, but Rumba can't do such tricks and need explicit connections.

        # * A custom driverInput export

        driverList_array = context.create_node(parent, "MakeArray", "array")
        context.connect(driverList_array.get_plug_path("array"), node.get_plug_path("driverList"))

        input = dagNodeFn.findPlug("input", True)
        inputIds = [i for i in input.getExistingArrayAttributeIndices()]
        node.set_attribute(context, "inputIds", ["BufferUInt32", inputIds])

        output = dagNodeFn.findPlug("output", True)
        outputIds = [i for i in output.getExistingArrayAttributeIndices()]
        node.set_attribute(context, "outputIds", ["BufferUInt32", outputIds])

        pose = dagNodeFn.findPlug("pose", True)
        poseIds = [i for i in pose.getExistingArrayAttributeIndices()]
        node.set_attribute(context, "poseIds", ["BufferUInt32", poseIds])

        restInput = dagNodeFn.findPlug("restInput", True)
        restInputIds = [i for i in restInput.getExistingArrayAttributeIndices()]
        node.set_attribute(context, "restInputIds", ["BufferUInt32", restInputIds])

        driverList = dagNodeFn.findPlug("driverList", True)
        logical_indices = driverList.getExistingArrayAttributeIndices()
        driver_count = 0 if len(logical_indices) == 0 else logical_indices[-1]+1
        for driverList_i in range(driver_count):
            driver_input = context.add_array_input(parent, driverList_array, driverList_i)
            if not driverList_i in logical_indices:
                continue

            driver = driverList.elementByLogicalIndex(driverList_i)
            driver_dict = context.create_node(parent, "MakeDict", "dict")
            context.connect(driver_dict.get_plug_path("dict"), driver_input)
            driver_key_list = []
            context.add_dict_input(driver_dict, "driverInput", driver_key_list)
            context.add_dict_input(driver_dict, "pose", driver_key_list)
            driverParentMatInv_input = context.add_dict_input(driver_dict, "driverParentMatInv", driver_key_list)
            driverParentJointOrient_input = context.add_dict_input(driver_dict, "driverParentJointOrient", driver_key_list)

            # * Normal export for driverInput and pose
            driverInput = driver.child(0)
            walk(context, parent, dagNodeFn, driverInput, driver_dict, "driverInput")
            pose = driver.child(2)
            walk(context, parent, dagNodeFn, pose, driver_dict, "pose", children=["poseMatrix", "poseParentMatrix", "poseMode"])

            # We need the pose's logical indices
            context.add_dict_input(driver_dict, "poseMatrixIds", driver_key_list)
            poseMatrixIds = [i for i in pose.getExistingArrayAttributeIndices()]
            driver_dict.set_attribute(context, "poseMatrixIds", ["BufferUInt32", poseMatrixIds])

            # Get the driverInput connection
            driverInput_connections = driverInput.connectedTo(True, False)
            if len(driverInput_connections) > 0:
                driverInput_obj = driverInput_connections[0].node()
                driverInput_node = MFnDagNode()
                if driverInput_node.hasObj(driverInput_obj):
                    driverInput_node.setObject(driverInput_obj)

                    # Connect parentInverseMatrix[first_index], the original plug-in does a getAPathTo to get this node, which is similar
                    parentInverseMatrix = driverInput_node.findPlug("parentInverseMatrix", True)
                    parentInverseMatrix_indices = parentInverseMatrix.getExistingArrayAttributeIndices()
                    if len(parentInverseMatrix_indices) > 0:
                        parentInverseMatrix0 = parentInverseMatrix.elementByPhysicalIndex(0)
                        parentInverseMatrix0_path = export_plug(context, parent, parentInverseMatrix0)
                        context.connect(parentInverseMatrix0_path, driverParentMatInv_input)

                    # Connect jointOrient if it exists
                    if driverInput_node.hasAttribute("jointOrient"):
                        jointOrient = driverInput_node.findPlug("jointOrient", True)
                        jointOrient_path = export_plug(context, parent, jointOrient)
                        context.connect(jointOrient_path, driverParentJointOrient_input)

            context.finish_dict(driver_dict, driver_key_list)

        #Input("driverList", "driverList", "Array", children=["driverInput", "driverParentMatInv", "driverParentJointOrient", "pose", "poseMatrix", "poseParentMatrix", "poseMode"]),

exporter = weightDriverExporter()