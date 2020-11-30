// ---------------------------------------------------------------------
//
//  weightDriver.cpp
//
//  Created by ingo on 9/27/13.
//  Copyright (c) 2013 Ingo Clemens. All rights reserved.
//  Ported to Rumba by Mercenaries Engineering SARL
//
// ---------------------------------------------------------------------

#include "weightDriver.h"
#include "DictOrArray.h"

#include "math.h"

#define DOUBLE_EPSILON 2.2204460492503131e-16

const float DEGTORAD = (float)(M_PI / 180);
const float RADTODEG = (float)(180 / M_PI);

#include "Rumba/Rumba.h"
#include <maya/MIntArray.h>

using namespace rumba;
using namespace Imath;

double getPoseDelta(std::vector<double> vec1, std::vector<double> vec2, int distType);
double getRadius(std::vector<double> vec1, std::vector<double> vec2);
double getAngle(std::vector<double> vec1, std::vector<double> vec2);
double interpolateRbf(double value, double width, short kernelType);

void buffer_to_array(const BufferConstUInt32& b, MIntArray& a)
{
    a.setLength((unsigned)b.size());
    const int n = (int)b.size();
    for(int i = 0; i < n; ++i)
        a[i] = b[i];
}

//
// Description:
//      Calculate the twist angle based on the given rotate order.
//
// Input Arguments:
//      q               The quaternion to get the twist angle from.
//      axis            The twist axis.
//
// Return Value:
//      double          The twist angle.
//
double getTwistAngle(MQuaternion q, unsigned int axis)
{
    double axisComponent = q.x;
    if (axis == 1)
        axisComponent = q.y;
    else if (axis == 2)
        axisComponent = q.z;
    return 2.0 * atan2(axisComponent, q.w);
}


//
// Description:
//      Build a matrix containing the distance values between all poses.
//      Based on all distances also calculate the general mean distance
//      which is needed for the RBF calculation.
//
// Input Arguments:
//      poseMat         The matrix containing all poses.
//      meanDist        The average distance between the poses.
//      distType        The distance type (linear/angle).
//
// Return Value:
//      BRMatrix        The twist angle.
//
BRMatrix getDistances(BRMatrix poseMat, double &meanDist, int distType)
{
    unsigned count = poseMat.getRowSize();

    unsigned int i, j;

    BRMatrix distMat;
    distMat.setSize(count, count);

    double sum = 0.0;

    for (i = 0; i < count; i ++)
    {
        for (j = 0; j < count; j ++)
        {
            double dist = getPoseDelta(poseMat.getRowVector(i), poseMat.getRowVector(j), distType);
            distMat(i, j) = dist;
            sum += dist;
        }
    }

    meanDist = sum / (count * count);

    return distMat;
}


//
// Description:
//      Return the distance between the two given vectors based on the
//      distance type (linear/angle) or the vector components in case
//      of the generic RBF mode.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//      distType        The distance type (linear/angle).
//
// Return Value:
//      double          The distance value.
//
double getPoseDelta(std::vector<double> vec1, std::vector<double> vec2, int distType)
{
    double dist = 0.0;
    if (distType == 0)
        dist = getRadius(vec1, vec2);
    else
    {
        if (vec1.size() == 3 && vec2.size() == 3)
            dist = getAngle(vec1, vec2);
        else
            dist = getRadius(vec1, vec2);
    }
    return dist;
}


//
// Description:
//      Calculate the linear distance between two vectors.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//
// Return Value:
//      double          The linear distance.
//
double getRadius(std::vector<double> vec1, std::vector<double> vec2)
{
    size_t count = vec1.size();

    double sum = 0.0;
    for (unsigned i = 0; i < count; i ++)
        sum += pow(vec1[i] - vec2[i], 2);
    return sqrt(sum);
}


//
// Description:
//      Calculate the angle between two vectors.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//
// Return Value:
//      double          The angle value.
//
double getAngle(std::vector<double> vec1, std::vector<double> vec2)
{
    MVector v1(vec1[0], vec1[1], vec1[2]);
    MVector v2(vec2[0], vec2[1], vec2[2]);
    return v1.angle(v2);
}


//
// Description:
//      Calculate the RBF activation values.
//
// Input Arguments:
//      mat             The matrix with the activation values.
//      width           The activation width.
//      kernelType      The interpolation function.
//
// Return Value:
//      None
//
void getActivations(BRMatrix &mat, double width, short kernelType)
{
    unsigned count = mat.getRowSize();

    unsigned int i, j;

    for (i = 0; i < count; i ++)
    {
        for (j = 0; j < count; j ++)
            mat(i, j) = interpolateRbf(mat(i, j), width, kernelType);
    }
}


//
// Description:
//      Interpolation function for processing the weight values.
//
// Input Arguments:
//      value           The value to interpolate.
//      width           The activation width.
//      kernelType      The interpolation function.
//
// Return Value:
//      double          The new interpolated value.
//
double interpolateRbf(double value, double width, short kernelType)
{
    double result = 0.0;

    // gaussian
    if (kernelType == 0)
    {
        if (width == 0.0)
            width = 1.0;
        width = 1.0 / width;
        double sigma = -(width * width);
        result = exp(sigma * value);
    }
    // linear
    else if (kernelType == 1)
    {
        result = value;
    }

    return result;
}


//
// Description:
//      Calculate the individual output weights based on the current
//      driver values in relation to the stored poses. This is the main
//      part of the RBF calculation but a rather simple process as it
//      just gets the distances of the driver to the stored poses and
//      calculates the weighted output values based on the weight matrix
//      built during initialization.
//
// Input Arguments:
//      out             The array of output weight values.
//      poses           The matrix containing all poses.
//      driver          The array of driver values.
//      poseModes       The array containing the the mode per pose.
//      weightMat       The matrix with the RBF weights.
//      avgDist         The average distance between the poses.
//      distType        The distance type (linear/angle).
//      kernelType      The interpolation function.
//
// Return Value:
//      None
//
void getPoseWeights(MDoubleArray &out,
                                  BRMatrix poses,
                                  std::vector<double> driver,
                                  MIntArray poseModes,
                                  BRMatrix weightMat,
                                  double avgDist,
                                  int distType,
                                  short kernelType)
{
    unsigned int poseCount = poses.getRowSize();
    unsigned int valueCount = out.length();

    // Make sure that the weight matrix has the correct dimensions.
    // This has become necessary with introducing multiple drivers in
    // matrix mode.
    if (weightMat.getRowSize() != poseCount || weightMat.getColSize() != valueCount)
        return;

    unsigned int i, j;

    for (i = 0; i < poseCount; i ++)
    {
        double dist = 0.0;
        std::vector<double> dv = driver;
        std::vector<double> ps = poses.getRowVector(i);

        if (poseModes[i] == 1)
            dv[3] = 0.0;
        else if (poseModes[i] == 2)
        {
            dv[0] = 0.0;
            dv[1] = 0.0;
            dv[2] = 0.0;
        }

        dist = getPoseDelta(dv, ps, distType);

        for (j = 0; j < valueCount; j ++)
            out[j] += weightMat(i, j) * interpolateRbf(dist, avgDist, kernelType);
    }
}


//
// Description:
//      Pass the weight values to the outputs.
//
// Input Arguments:
//      weightsArray    The array of output weight values.
//      data            The MPxNode dataBlock.
//      inactive        True, if the node is enabled.
//
// Return Value:
//      None
//
void setOutputValues(MDoubleArray weightsArray, Array& output, bool inactive, const MIntArray& poseMatrixIds, bool genericMode)
{
    unsigned int i;

    // In generic mode pose and output indices are not related.
    // The ordering of the output always starts at 0 with an increment
    // of 1, no matter if pose indices are missing.
    // In matrix mode pose and output indices are matching, due to the
    // square dimensions of blendshape usage.
    unsigned count = 0;
    MIntArray ids;
    if (genericMode)
    {
        count = weightsArray.length();
        ids.setLength(count);
        for (i = 0; i < count; i ++)
            ids.set((int)i, i);
    }
    else
    {
        count = poseMatrixIds.length();
        ids = poseMatrixIds;
    }

    for (i = 0; i < count; i ++)
    {
        while(int(output.size()) < ids[i])
            output.push_back(0.f);
        if (!inactive)
            output.push_back(float(weightsArray[i]));
        else
            output.push_back(0.f);
    }
}


//
// Description:
//      Modify the value by shifting it towards the lower or upper end
//      of the interpolation range.
//
// Input Arguments:
//      value           The value to shift.
//      biasValue       The bias value.
//
// Return Value:
//      double          The new value with bias.
//
double rbfWeightBias(double value, double biasValue)
{
    if (biasValue >= 0.0)
    {
        value = fabs(value) * pow(fabs(value), biasValue);
    }
    else
    {
        double baseVal = 1 - fabs(value);
        // If the value is 1 the bias transformation has zero at the
        // base and outputs NaN.
        if (fabs(baseVal) > DOUBLE_EPSILON)
            value = 1 - pow(baseVal, (1 + fabs(biasValue)));
        else
            value = 1;
    }

    return value;
}

//
// Description:
//      Return the blend curve weight value at the given position.
//
// Input Arguments:
//      value           The input value of the blend curve.
//
// Return Value:
//      double          The blend curve output value.
//
double blendCurveWeight(double value, MRampAttribute& curveAttr)
{
    float curveValue;
    curveAttr.getValueAtPosition((float)value, curveValue);
    value = curveValue;

    return value;
}

//
// Description:
//      Modify the output weight value by the chosen interpolation type.
//
// Input Arguments:
//      value           The value to interpolate.
//      type            The type of interpolation.
//
// Return Value:
//      double          The new interpolated value.
//
double interpolateWeight(double value, int type, MRampAttribute& curveAttr)
{
    // slow - inverse quadratic
    if (type == 1)
        value = 1 - pow((1 - value), 2.0);
    // fast - quadratic
    else if (type == 2)
        value = 1 - pow((1 - value), 1 / 2.0);
    // smooth1 - smoothstep
    else if (type == 3)
        value = value * value * (3 - 2 * value);
    // smooth2 - smootherstep
    else if (type == 4)
        value = value * value * value * (value * (value * 6 - 15) + 10);
    else if (type == 5)
        value = blendCurveWeight(value, curveAttr);

    return value;
}

enum OutputPlug
{
    output = 0,
    outWeight
};

enum
{
    active = 0,
    allowNegativeWeights,
    angle,
    bias,
    centerAngle,
    blendCurve,
    direction,
    distanceType,
    driverIndex,
    driverMatrix,
    evaluate,
    grow,
    input,
    interpolation,
    invert,
    kernel,
    opposite,
    scale,
    rbfMode,
    readerMatrix,
    restInput,
    translateMax,
    translateMin,
    twist,
    twistAngle,
    twistAxis,
    type,
    useInterpolation,
    useRotate,
    useTranslate,
    poses,
    driverList,
    _inputIds,
    _outputIds,
    _poseIds,
    _restInputIds,
};

//
// Description:
//      Collect all driver and pose relevant data.
//      RBF Matrix Mode (when using SHAPES)
//
// Input Arguments:
//      data            The MPxNode dataBlock.
//      driver          The array of driver values. Each driver has four
//                      values: the vector and the twist value. The
//                      array length is numberOfDrivers * 4.
//      poseCount       The number of poses.
//      poseData        The matrix containing all poses.
//      poseVals        The matrix of pose values.
//      poseModes       The array containing the the mode per pose.
//      twistAxisVal    The twist axis.
//      invertAxes      True, if the axis should be inverted.
//      driverId        The index of the driver for drawing.
//
// Return Value:
//      MStatus
//
void getPoseVectors(EvalContext& ctx,
    std::vector<double> &driver,
    unsigned &poseCount,
    BRMatrix &poseData,
    BRMatrix &poseVals,
    MIntArray &poseModes,
    unsigned twistAxisVal,
    bool invertAxes,
    unsigned driverId,
    MIntArray& poseMatrixIds
    )
{
    unsigned int d, i, p;
    unsigned increment = 0;

    // -----------------------------------------------------------------
    // create the base vector
    // -----------------------------------------------------------------

    MVector baseVec(1.0, 0.0, 0.0);
    // Define the reference vector base.
    if (twistAxisVal == 1)
        baseVec = MVector(0.0, 1.0, 0.0);
    else if (twistAxisVal == 2)
        baseVec = MVector(0.0, 0.0, 1.0);

    if (invertAxes)
        baseVec *= -1;

    // -----------------------------------------------------------------
    // get the driver list handle
    // -----------------------------------------------------------------

    const Array driverListHandle = ctx.value(driverList);
    unsigned driverCount = (unsigned)driverListHandle.size();

    // -----------------------------------------------------------------
    // process for each driver
    // -----------------------------------------------------------------

    for (d = 0; d < driverCount; d ++)
    {
        const Dict driverListIdHandle = driverListHandle.read(d);

        // -------------------------------------------------------------
        // get the attributes
        // -------------------------------------------------------------

        MMatrix driverMat = driverListIdHandle.as_M44f("driverInput");
        MMatrix driverParentMatInv = driverListIdHandle.as_M44f("driverParentMatInv");
        const V3f jointOrient = driverListIdHandle.as_V3f("driverParentJointOrient");
        const MMatrix jointOrientMatInv = Eulerf(jointOrient, Eulerf::Order::XYZ).toMatrix44().inverse();

        const Array poseArrayHandle(driverListIdHandle.read("pose"));
        {
            const BufferConstUInt32 _poseMatrixIds(driverListIdHandle.read("poseMatrixIds"));
            poseMatrixIds.clear();
            for(const auto index : _poseMatrixIds)
                poseMatrixIds.append(index);
        }

        // Build a local transform matrix.
        MTransformationMatrix transMatDriver = driverMat * driverParentMatInv * jointOrientMatInv;

        MQuaternion quatDriver = transMatDriver.rotation();

        // -------------------------------------------------------------
        // create the driver vector
        // -------------------------------------------------------------

        MVector driverMVec = baseVec * transMatDriver.asMatrix();
        MVector driverMVecDraw = baseVec * driverMat;

        // -------------------------------------------------------------
        // set the driver vector and twist
        // -------------------------------------------------------------

        driver[0 + increment] = driverMVec.x;
        driver[1 + increment] = driverMVec.y;
        driver[2 + increment] = driverMVec.z;
        driver[3 + increment] = getTwistAngle(quatDriver, twistAxisVal);

        // -------------------------------------------------------------
        // get the pose array indices and set the matrices
        // -------------------------------------------------------------

        // Do this only for the first driver because even if there is
        // more than one driver all other drivers should have the same
        // amount of poses and data values.
        if (d == 0)
        {
            /*posePlug.getExistingArrayAttributeIndices(poseMatrixIds, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);*/

            poseCount = poseMatrixIds.length();

            /*if (poseCount != globalPoseCount)
            {
                globalPoseCount = poseCount;
                evalInput = true;
            }*/

            // ---------------------------------------------------------
            // prepare the data matrices
            // ---------------------------------------------------------

            // Prepare the matrix to hold the pose vectors.
            // Assign an empty matrix to clear pre-existing data.
            poseData = BRMatrix();
            poseData.setSize(poseCount, 4 * driverCount);

            // Prepare the matrix to hold the pose values.
            // Assign an empty matrix to clear pre-existing data.
            poseVals = BRMatrix();
            poseVals.setSize(poseCount, poseCount);
        }

        // -------------------------------------------------------------
        // get the pose matrices and define the pose vectors
        // -------------------------------------------------------------

        if (poseCount)
        {
            // ---------------------------------------------------------
            // prepare the data matrices
            // ---------------------------------------------------------

            MVectorArray poseVectors;
            poseVectors.setLength(poseCount);
            MDoubleArray poseTwist;
            poseTwist.setLength(poseCount);
            MVectorArray poseVectorsDraw;
            poseVectorsDraw.setLength(poseCount);

            // Copy the previous pose modes for comparison to see
            // if the matrices need to get updated.
            MIntArray poseModesPrev = poseModes;

            // Clear pre-existing pose modes.
            poseModes.clear();
            poseModes.setLength(poseCount);

            // ---------------------------------------------------------
            // get the pose data
            // ---------------------------------------------------------

            for (i = 0; i < poseCount; i ++)
            {
                // status = poseArrayHandle.jumpToArrayElement(i);
                const DictOrArray poseIdHandle = poseArrayHandle.read(poseMatrixIds[i]);
                MMatrix poseMat = poseIdHandle.as_M44f("poseMatrix", 0);
                MMatrix parentMat = poseIdHandle.as_M44f("poseParentMatrix", 1);

                MMatrix poseMatRel = poseMat * parentMat.inverse() * jointOrientMatInv;

                // -----------------------------------------------------
                // pose mode
                // -----------------------------------------------------

                int poseModeValue = poseIdHandle.as_int("poseMode", 2);
                poseModes.set(poseModeValue, i);

                // Evaluation for the processing the matrices always
                // needs to be active when the pose mode for a pose
                // changes.
                /* if (poseModesPrev.length() && poseModeValue != poseModesPrev[i])
                    evalInput = true; */

                // -----------------------------------------------------
                // pose vectors
                // -----------------------------------------------------

                MVector poseVec = baseVec * poseMatRel;
                poseVectors.set(poseVec, i);

                MVector poseVecDraw = baseVec * poseMat;
                poseVectorsDraw.set(poseVecDraw, i);

                // -----------------------------------------------------
                // pose vector and twist angle
                // -----------------------------------------------------

                MTransformationMatrix transMatPose = poseMatRel;
                MQuaternion quatPose = transMatPose.rotation();

                if (poseModes[i] != 2)
                {
                    poseData(i, 0 + increment) = poseVec.x;
                    poseData(i, 1 + increment) = poseVec.y;
                    poseData(i, 2 + increment) = poseVec.z;
                }

                poseData(i, 3 + increment) = 0.0;
                poseTwist.set(0.0, i);
                if (poseModes[i] != 1)
                {
                    double twistVal = getTwistAngle(quatPose, twistAxisVal);
                    poseData(i, 3 + increment) = twistVal;
                    poseTwist.set(twistVal, i);
                }

                // -----------------------------------------------------
                // pose values
                // -----------------------------------------------------

                // Create the vector for the pose values.
                if (d == 0)
                {
                    for (p = 0; p < poseCount; p ++)
                    {
                        poseVals(i, p) = 0;
                        if (i == p)
                            poseVals(i, p) = 1;
                    }
                }
            }

            // ---------------------------------------------------------
            // fill the array for drawing
            // ---------------------------------------------------------

            /* if (d == driverId)
            {
                // Copy the pose vectors and twist for the legacy
                // viewport display.
                poseVectorArray.copy(poseVectorsDraw);
                poseVectorArray.append(driverMVecDraw);
                poseTwistArray.copy(poseTwist);
                poseTwistArray.append(driver[3 + increment]);

                // Copy the pose vectors and twist values for the VP 2.0
                // display.
                MArrayDataHandle pvHandle = data.outputArrayValue(poseDrawVector);
                MArrayDataBuilder pvBuilder(&data, poseDrawVector, poseCount + 1);
                MArrayDataHandle ptHandle = data.outputArrayValue(poseDrawTwist);
                MArrayDataBuilder ptBuilder(&data, poseDrawTwist, poseCount + 1);
                for (i = 0; i < poseCount; i ++)
                {
                    MDataHandle pvIdHandle = pvBuilder.addElement((unsigned)poseMatrixIds[i]);
                    pvIdHandle.set3Double(poseVectorsDraw[i].x, poseVectorsDraw[i].y, poseVectorsDraw[i].z);
                    pvHandle.set(pvBuilder);
                    pvHandle.setAllClean();

                    MDataHandle ptIdHandle = ptBuilder.addElement((unsigned)poseMatrixIds[i]);
                    ptIdHandle.setDouble(poseData(i, 3 + increment));
                    ptHandle.set(ptBuilder);
                    ptHandle.setAllClean();
                }
                // Add the driver vector.
                MDataHandle pvIdHandle = pvBuilder.addElement((unsigned)poseMatrixIds[poseCount - 1] + 1);
                pvIdHandle.set3Double(driverMVecDraw.x, driverMVecDraw.y, driverMVecDraw.z);
                pvHandle.set(pvBuilder);
                pvHandle.setAllClean();

                // Add the driver twist.
                MDataHandle ptIdHandle = ptBuilder.addElement((unsigned)poseMatrixIds[poseCount - 1] + 1);
                ptIdHandle.setDouble(driver[3 + increment]);
                ptHandle.set(ptBuilder);
                ptHandle.setAllClean();
            }*/
        }

        increment += 4;
    }
}

//
// Description:
//      Collect all driver and pose relevant data.
//      Generic Mode
//
// Input Arguments:
//      data            The MPxNode dataBlock.
//      driver          The array of driver values.
//      poseCount       The number of poses.
//      solveCount      The number of outputs to generate values for.
//      poseData        The matrix containing all poses.
//      poseVals        The matrix of pose values.
//      poseModes       The array containing the the mode per pose.
//
// Return Value:
//      MStatus
//
void getPoseData(EvalContext& ctx,
                    std::vector<double> &driver,
                    unsigned &poseCount,
                    unsigned &solveCount,
                    BRMatrix &poseData,
                    BRMatrix &poseVals,
                    MIntArray &poseModes,
                    MIntArray &poseMatrixIds,
                    int distanceTypeVal)
{
    unsigned int i, j;

    // -----------------------------------------------------------------
    // get the number of outputs
    // -----------------------------------------------------------------

    const BufferConstUInt32 outputIds = ctx.value(_outputIds);
    solveCount = (unsigned)outputIds.size();

    // -----------------------------------------------------------------
    // get the data handles
    // -----------------------------------------------------------------

    const Array inputHandle = ctx.value(input);

    const Array restInputHandle = ctx.value(restInput);

    const Array posesHandle = ctx.value(poses);

    // -----------------------------------------------------------------
    // get the array ids
    // -----------------------------------------------------------------

    MIntArray inputIds;
    MIntArray restInputIds;
    MIntArray poseIds;

    const BufferConstUInt32 __inputIds = ctx.value(_inputIds);
    buffer_to_array(__inputIds, inputIds);

    const BufferConstUInt32 __restInputIds = ctx.value(_restInputIds);
    buffer_to_array(__restInputIds, restInputIds);

    const BufferConstUInt32 __poseIds = ctx.value(_poseIds);
    buffer_to_array(__poseIds, poseIds);

    unsigned inDim = inputIds.length();
    unsigned restDim = restInputIds.length();

    poseCount = poseIds.length();
    // Store the original pose count before the count gets modified
    // because of a missing 0 index.
    // The original index list is important when querying the last index
    // of the array, see below *).
    unsigned poseCountOriginal = poseCount;

    // Make sure to start at a pose index of 0.
    // Because Maya creates sparse arrays it's possible that the first
    // pose gets lost when a rest pose is present which only contain
    // zero values.
    if (poseCount != 0 && poseIds[0] != 0)
    {
        poseIds.insert(0, 0);
        poseCount ++;
    }
    // Problem: *)
    // When loading a scene with the weightDriver node the index count
    // of the poses plug (compound array attribute) matches the number
    // of poses, whereas once the scene gets evaluated the plug array
    // contains an additional empty (next available) index.
    // Since the correct number of poses needs to be known for creating
    // the matrices, the last index gets checked. If the child
    // attributes have elements in case of a freshly loaded scene, the
    // pose count doesn't need to be altered. But when the scene already
    // has been evaluated the children of the last index don't have any
    // elements and therefore can be ignored.
    // posesHandle.jumpToArrayElement(poseCountOriginal - 1);
    const DictOrArray lastIdHandle = posesHandle.read(poseIds[std::min(poseIds.length()-1, poseCountOriginal - 1)]);
    const Array lastInputArrayHandle = lastIdHandle.read("poseInput", 0);
    unsigned lastInCount = (unsigned)lastInputArrayHandle.size();
    if (lastInCount == 0)
        poseCount --;

    // Check for any pose connections. In case the pose attributes are
    // connected all data need to get re-evaluated, which slows down the
    // calculation.
    /* unsigned int numConnChildren = posesPlug.numConnectedChildren();
    if (numConnChildren != 0 || poseCount != globalPoseCount)
        evalInput = true; */

    // Clear the indices for setting the output array values because
    // valid indices get appended.
    poseMatrixIds.clear();

    // -----------------------------------------------------------------
    // fill the driver and rest vector
    // -----------------------------------------------------------------

    std::vector<double> rest;
    rest.resize(inDim);

    for (i = 0; i < inDim; i ++)
    {
        driver[i] = inputHandle.read(inputIds[i]).as_float();

        // get the rest input
        if (i < restDim)
        {
            rest[i] = restInputHandle.read(restInputIds[i]).as_float();
        }
        else
            rest[i] = 0.0;

        if (distanceTypeVal)
            driver[i] -= rest[i];
        else
            rest[i] = 0.0;
    }

    // -----------------------------------------------------------------
    // get the pose data
    // -----------------------------------------------------------------

    if (poseCount != 0 /* && evalInput */)
    {
        // globalPoseCount = poseCount;

        // Prepare the matrix to hold the pose vectors.
        // Assign an empty matrix to clear pre-existing data.
        poseData = BRMatrix();
        poseData.setSize(poseCount, inDim);

        // Prepare the matrix to hold the pose values.
        // Assign an empty matrix to clear pre-existing data.
        poseVals = BRMatrix();
        poseVals.setSize(poseCount, solveCount);

        // Clear pre-existing mode modes.
        poseModes.clear();
        poseModes.setLength(poseCount);

        for (i = 0; i < poseCount; i ++)
        {
            poseMatrixIds.append((int)i);

            // Fill with zeros in case an array index is missing because
            // of sparse arrays.
            for (j = 0; j < inDim; j ++)
                poseData(i, j) = 0.0 - rest[j];
            for (j = 0; j < solveCount; j ++)
                poseVals(i, j) = 0.0;

            // ---------------------------------------------------------
            // pose positions
            // ---------------------------------------------------------

            {
                const DictOrArray posesIdHandle = posesHandle.read(poseIds[i]);
                const Array poseInputArrayHandle = posesIdHandle.read("poseInput", 0);

                unsigned poseInputCount = (unsigned)poseInputArrayHandle.size();

                for (j = 0; j < inDim; j ++)
                {
                    // Handle the special case of sparse arrays which
                    // might hold less data than is needed.
                    if (poseInputCount != 0)
                    {
                        // status = poseInputArrayHandle.jumpToElement(j);
                        if(j < (unsigned)poseInputArrayHandle.size())
                            poseData(i, j) = poseInputArrayHandle.read(j).as_float() - rest[j];
                    }
                }

                // -----------------------------------------------
                // pose values
                // -----------------------------------------------

                const Array poseValueArrayHandle = posesIdHandle.read("poseValue", 1);

                unsigned valueCount = (unsigned)poseValueArrayHandle.size();

                for (j = 0; j < solveCount; j ++)
                {
                    // Handle the special case of sparse arrays which
                    // might hold less data than is needed.
                    if (valueCount != 0)
                    {
                        // status = poseValueArrayHandle.jumpToElement(j);
                        if (j < (unsigned)poseValueArrayHandle.size())
                            poseVals(i, j) = poseValueArrayHandle.read(j).as_float();
                    }
                }
            }

            // -----------------------------------------------
            // pose modes
            // -----------------------------------------------

            // Set the pose mode value. This is not necessary for
            // generic mode, but only to make the data for both modes
            // consistent.
            poseModes.set(0, i);
        }
    }
}

rumba::Value compute(OutputPlug plug, EvalContext& ctx)
{
    // -----------------------------------------------------------------
    // get the attributes
    // -----------------------------------------------------------------
    bool activeVal = ctx.as_bool(active);
    bool allowNegativeVal = ctx.as_bool(allowNegativeWeights);
    double angleVal = ctx.as_float(angle);
    double biasVal = ctx.as_float(bias);
    double centerAngleVal = ctx.as_float(centerAngle);
    int dirVal = ctx.as_int(direction);
    int distanceTypeVal = ctx.as_int(distanceType);
    int driverIndexVal = ctx.as_int(driverIndex);
    bool evalInput = ctx.as_bool(evaluate);
    bool growVal = ctx.as_bool(grow);
    short interVal = ctx.as_int(interpolation);
    bool invVal = ctx.as_bool(invert);
    short kernelVal = ctx.as_int(kernel);
    bool oppositeVal = ctx.as_bool(opposite);
    double scaleVal = ctx.as_float(scale);
    double twistAngleVal = ctx.as_float(twistAngle);
    short twistAxisVal = ctx.as_int(twistAxis);
    bool twistVal = ctx.as_bool(twist);
    double translateMaxVal = ctx.as_float(translateMax);
    double translateMinVal = ctx.as_float(translateMin);
    bool useInterpolationVal = ctx.as_bool(useInterpolation);
    bool useRotateVal = ctx.as_bool(useRotate);
    bool useTranslateVal = ctx.as_bool(useTranslate);
    int typeVal = ctx.as_int(type);
    const Array blendCurveVal = ctx.value(blendCurve);

    MRampAttribute curveAttr(blendCurveVal, "blendCurve");
    bool genericMode = false;
    MIntArray poseMatrixIds;

    if (((plug == output && typeVal != 0) || (plug == outWeight && typeVal == 0)) && activeVal)
    {
        // -------------------------------------------------------------
        // main calculation
        // -------------------------------------------------------------

        MDoubleArray weightsArray;
        unsigned poseCount = 1;

        // -------------------------------------------------------------
        // vector angle
        // -------------------------------------------------------------

        if (typeVal == 0)
        {
            // ---------------------------------------------------------
            // get the general matrix data handles
            // ---------------------------------------------------------

            MMatrix readerMat = ctx.as_M44f(readerMatrix);

            MMatrix driverMat = ctx.as_M44f(driverMatrix);

            MTransformationMatrix transMatReader = readerMat;
            MTransformationMatrix transMatDriver = driverMat;

            MVector readerPos = transMatReader.getTranslation(MSpace::kWorld);
            MVector driverPos = transMatDriver.getTranslation(MSpace::kWorld);

            MVector driverMVec = driverPos - readerPos;

            weightsArray.setLength(poseCount);

            // ---------------------------------------------------------
            // define the target vector
            // ---------------------------------------------------------

            MPoint targetPos;
            MVector upMVec;

            double axis = 1.0;
            if (invVal)
                axis = -1.0;

            if (dirVal == 0)
            {
                targetPos = MPoint(axis, 0.0, 0.0);
                upMVec = MVector(0.0, 1.0, 0.0);
            }
            else if (dirVal == 1)
            {
                targetPos = MPoint(0.0, axis, 0.0);
                upMVec = MVector(1.0, 0.0, 0.0);
            }
            else
            {
                targetPos = MPoint(0.0, 0.0, axis);
                upMVec = MVector(1.0, 0.0, 0.0);
            }

            targetPos *= readerMat;

            MVector targetMVec = targetPos - readerPos;
            MMatrix relativeMat = readerMat * driverMat.inverse();

            // ---------------------------------------------------------
            // calculate the twist value
            // ---------------------------------------------------------

            double twistWeightVal = 1.0;

            if (twistVal)
            {
                MVector twistMVec = upMVec * relativeMat;
                twistMVec.normalize();

                double twistAngle = twistMVec.angle(upMVec);
                twistAngle = twistAngle * RADTODEG;

                twistWeightVal = 1 - twistAngle / twistAngleVal;
            }

            // ---------------------------------------------------------
            // calculate the translate value
            // ---------------------------------------------------------

            double translateVal = 1;

            if (useTranslateVal)
            {
                MTransformationMatrix transMatRelative = relativeMat;
                MVector transMVec = transMatRelative.getTranslation(MSpace::kWorld);
                double distance = transMVec.length();
                if (distance <= translateMinVal)
                    translateVal = 1;
                else if (distance >= translateMaxVal)
                    translateVal = 0;
                else
                {
                    translateVal = 1 - ((distance - translateMinVal)
                                   / (translateMaxVal - translateMinVal));
                }

                if (growVal)
                    translateVal = 1 - translateVal;
            }

            // ---------------------------------------------------------
            // calculate the vectors and resulting angle
            // ---------------------------------------------------------

            double weightVal = 1;

            if (useRotateVal)
            {
                double offsetAngle = targetMVec.angle(driverMVec);
                offsetAngle = offsetAngle * RADTODEG;

                weightVal = 1 - offsetAngle / angleVal;

                weightVal *= twistWeightVal;

                // Make sure that the center angle is always smaller
                // than the angle.
                if (angleVal <= centerAngleVal)
                    centerAngleVal = angleVal - 0.1;

                // Create another value from the center angle to
                // calculate an offset factor for widening the center
                // range.
                double centerVal = (angleVal - centerAngleVal) / angleVal;
                weightVal /= centerVal;
            }

            weightVal *= translateVal;

            // Clamp the value to a 0-1 range.
            if (weightVal <= 0)
                weightVal = 0;
            else if (weightVal >= 1)
                weightVal = 1;

            // ---------------------------------------------------------
            // apply the interpolation
            // ---------------------------------------------------------

            weightVal = interpolateWeight(weightVal, interVal, curveAttr);

            // ---------------------------------------------------------
            // set the output values
            // ---------------------------------------------------------

            // Pass the weight to the array output.
            weightsArray.set(weightVal, 0);

            // Pass the weight to the legacy outWeight plug.
            return float(weightsArray[0]);
        }

        // -------------------------------------------------------------
        // radial basis function
        // -------------------------------------------------------------

        else
        {
            unsigned int i, c;

            std::vector<double> driver;
            unsigned int solveCount;
            unsigned int driverCount = 0;

            // ---------------------------------------------------------
            // Check the rbf mode.
            // Any connected input assumes generic mode which is mainly
            // for switching the display of the locator.
            // ---------------------------------------------------------

            Array _inputIds = ctx.value(input);

            // Set generic mode to be the default.
            genericMode = true;
            if (_inputIds.size())
            {
/*                MDataHandle rbfModeHandle = ctx.outputValue(rbfMode);
                rbfModeHandle.set(0);*/
                genericMode = true;
            }
            else
            {
/*                MDataHandle rbfModeHandle = ctx.outputValue(rbfMode);
                rbfModeHandle.set(1);*/
                genericMode = false;
            }

            // ---------------------------------------------------------
            // get the pose data based on the mode
            // ---------------------------------------------------------

            BRMatrix matPoses;
            BRMatrix matValues;
            MIntArray poseModes;
            MIntArray inputIds;
            MIntArray poseIds;

            if (genericMode)
            {
                // genericMode not supported right now
                unsigned int driverSize = (unsigned int)_inputIds.size();
                driver.resize(driverSize);

                getPoseData(ctx,
                    driver,
                    poseCount,
                    solveCount,
                    matPoses,
                    matValues,
                    poseModes,
                    poseMatrixIds,
                    distanceTypeVal);
            }
            else
            {
                // get the driver indices
                const int driverIds_length = (int)Array(ctx.value(driverList)).size();

                driverCount = driverIds_length;
                driver.resize(4 * driverCount);

                getPoseVectors(ctx,
                    driver,
                    poseCount,
                    matPoses,
                    matValues,
                    poseModes,
                    (unsigned)twistAxisVal,
                    oppositeVal,
                    (unsigned)driverIndexVal,
                    poseMatrixIds);

                // Override the distance type for the matrix mode to
                // euclidean which makes it easier to calculate rotation
                // and twist.
                distanceTypeVal = 0;

                solveCount = poseCount;
            }

            /* if (exposeDataVal == 1 || exposeDataVal == 4)
            {
                matPoses.show(thisName, "Poses");
                matValues.show(thisName, "Values");
                //BRMatrix().showVector(driver, "driver");
            }*/

            // ---------------------------------------------------------
            // rbf calculation
            // ---------------------------------------------------------

            if (poseCount != 0)
            {
                // Set the default values for the output.
                weightsArray.setLength(solveCount);
                for (i = 0; i < solveCount; i ++)
                    weightsArray.set(0.0, i);

                BRMatrix wMat;
                double meanDist = 0.0;
                if (true /* evalInput */)
                {
                    // -------------------------------------------------
                    // distances
                    // -------------------------------------------------

                    // Create a distance matrix from all poses and
                    // calculate the mean distance for the rbf function.
                    BRMatrix linMat;
                    linMat = getDistances(matPoses, meanDist, distanceTypeVal);

                    /* if (exposeDataVal > 2)
                        linMat.show(thisName, "Distance matrix"); */

                    // -------------------------------------------------
                    // activations
                    // -------------------------------------------------

                    // Transform the distance matrix to include the
                    // activation values.
                    getActivations(linMat, meanDist, kernelVal);

                    /* if (exposeDataVal > 2)
                        linMat.show(thisName, "Activations"); */

                    // -------------------------------------------------
                    // prepare
                    // -------------------------------------------------

                    // create the matrix to store the weights
                    wMat = BRMatrix();
                    wMat.setSize(poseCount, solveCount);

                    // -------------------------------------------------
                    // solve for each dimension
                    // -------------------------------------------------

                    for (c = 0; c < solveCount; c ++)
                    {
                        // Get the pose values for each dimension.
                        std::vector<double> y;
                        y = matValues.getColumnVector(c);
                        //BRMatrix().showVector(y, MString(MString("values ") + c));

                        // Copy the activation matrix because it gets
                        // modified during the solving process.
                        BRMatrix solveMat = linMat;

                        double* w = new double [poseCount];
                        bool solved = solveMat.solve(y, w);
                        if (!solved)
                        {
                            //MGlobal::displayError("RBF decomposition failed");
                            // return MStatus::kFailure;
                        }

                        // Store the weights in the weight matrix.
                        for (i = 0; i < poseCount; i ++)
                            wMat(i, c) = w[i];

                        delete [] w;
                    }

                    /* if (exposeDataVal > 2)
                        wMat.show(thisName, "Weight matrix");*/
                }

                // -----------------------------------------------
                // final weight calculation
                // -----------------------------------------------

                getPoseWeights(weightsArray,
                               matPoses,
                               driver,
                               poseModes,
                               wMat,
                               meanDist,
                               distanceTypeVal,
                               kernelVal);

                /* 
                if (exposeDataVal == 2 || exposeDataVal == 4)
                    showArray(weightsArray, thisName + " : RBF Weights");
                    */

                // -----------------------------------------------
                // define the final values
                // -----------------------------------------------

                for (i = 0; i < weightsArray.length(); i ++)
                {
                    double value = weightsArray[i];

                    if (value < 0.0 && !allowNegativeVal)
                        value = 0.0;

                    if (biasVal != 0.0)
                        value = rbfWeightBias(value, biasVal);

                    if (useInterpolationVal)
                        value = interpolateWeight(value, interVal, curveAttr);

                    // Set the final weight.
                    weightsArray.set(value * scaleVal, i);
                }
            }
            // In case there are no poses generate a default value at
            // the output.
            else
            {
                weightsArray.setLength(1);
                weightsArray.set(1.0, 0);
            }
        }

        // -----------------------------------------------
        // pass the pose value to the output
        // -----------------------------------------------

        Array output;
        setOutputValues(weightsArray, output, false, poseMatrixIds, genericMode);

        return output;
    }
    else if (plug == output && !activeVal)
    {
        Array output;
        MDoubleArray da(1);
        da[0] = 0.0;
        setOutputValues(da, output, true, poseMatrixIds, genericMode);

        return output;
    }

    return Value::default_value;
}

static Value eval_output(EvalContext& ctx)
{
    return compute(output, ctx);
}

static Value eval_outWeight(EvalContext& ctx)
{
    return compute(outWeight, ctx);
}

void register_weightDriver( Registry &r )
{
    r.register_node
    ( 
        "weightDriver",
        "Node",
        {
            { "direction", 0, PlugDescriptor::serial, "{\"enum\":{\"X\":0,\"Y\":1,\"Z\":2}}" },
            { "distanceType", 0, PlugDescriptor::serial, "{\"enum\":{\"Euclidean\":0,\"Angle\":1}}" },
            { "interpolation", 0, PlugDescriptor::serial, "{\"enum\":{\"Linear\":0,\"Slow\":1,\"Fast\":2,\"Smooth1\":3,\"Smooth2\":4,\"Curve\":5}}" },
            { "kernel", 0, PlugDescriptor::serial, "{\"enum\":{\"Gaussian\":0,\"Linear\":1}}" },
            { "rbfMode", 0, PlugDescriptor::serial, "{\"enum\":{\"Generic\":0,\"Matrix\":1}}" },
            { "twistAxis", 0, PlugDescriptor::serial, "{\"enum\":{\"X\":0,\"Y\":1,\"Z\":2}}" },
            { "type", 0, PlugDescriptor::serial, "{\"enum\":{\"Vector Angle\":0,\"RBF\":1}}" },
            { "active", true },
            { "allowNegativeWeights", true },
            { "angle", 45.f, PlugDescriptor::serial, "{\"min\":0.01,\"max\":180.0}" },
            { "bias", 0.f, PlugDescriptor::serial, "{\"slider_min\":-1.0,\"slider_max\":1.0}" },
            { "centerAngle", 0.f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":180.0}" },
            { "driverIndex", 0 },
            { "evaluate", false },
            { "grow", true },
            { "input", Array::default_value },
            { "invert", false },
            { "opposite", false },
            { "restInput", Array::default_value },
            { "scale", 1.f },
            { "translateMax", 0.f, PlugDescriptor::serial, "{\"min\":0.0}" },
            { "translateMin", 0.f, PlugDescriptor::serial, "{\"min\":0.0}" },
            { "twist", false },
            { "twistAngle", 90.f, PlugDescriptor::serial, "{\"min\":0.01,\"max\":180.0}" },
            { "useInterpolation", true },
            { "useRotate", true },
            { "useTranslate", false },
            { "driverMatrix", identity44f },
            { "readerMatrix", identity44f },
            { "blendCurve", Array::default_value },
            { "poses", Array::default_value },
            { "driverList", Array::default_value },
            { "inputIds", BufferConstUInt32::default_value },
            { "outputIds", BufferConstUInt32::default_value },
            { "poseIds", BufferConstUInt32::default_value },
            { "restInputIds", BufferConstUInt32::default_value },
            { "output", Array::default_value, 0, "",
                eval_output,
                {
                    { "active" },
                    { "allowNegativeWeights" },
                    { "angle" },
                    { "bias" },
                    { "centerAngle" },
                    { "blendCurve" },
                    { "direction" },
                    { "distanceType" },
                    { "driverIndex" },
                    { "driverMatrix" },
                    { "evaluate" },
                    { "grow" },
                    { "input" },
                    { "interpolation" },
                    { "invert" },
                    { "kernel" },
                    { "opposite" },
                    { "scale" },
                    { "rbfMode" },
                    { "readerMatrix" },
                    { "restInput" },
                    { "translateMax" },
                    { "translateMin" },
                    { "twist" },
                    { "twistAngle" },
                    { "twistAxis" },
                    { "type" },
                    { "useInterpolation" },
                    { "useRotate" },
                    { "useTranslate" },
                    { "poses" },
                    { "driverList" },
                    { "inputIds" },
                    { "outputIds" },
                    { "poseIds" },
                    { "restInputIds" },
                }
            },
            { "outWeight", 0.f, 0, "",
                eval_outWeight,
                {
                    { "active" },
                    { "allowNegativeWeights" },
                    { "angle" },
                    { "bias" },
                    { "centerAngle" },
                    { "blendCurve" },
                    { "direction" },
                    { "distanceType" },
                    { "driverIndex" },
                    { "driverMatrix" },
                    { "evaluate" },
                    { "grow" },
                    { "input" },
                    { "interpolation" },
                    { "invert" },
                    { "kernel" },
                    { "opposite" },
                    { "scale" },
                    { "rbfMode" },
                    { "readerMatrix" },
                    { "restInput" },
                    { "translateMax" },
                    { "translateMin" },
                    { "twist" },
                    { "twistAngle" },
                    { "twistAxis" },
                    { "type" },
                    { "useInterpolation" },
                    { "useRotate" },
                    { "useTranslate" },
                    { "poses" },
                    { "driverList" },
                    { "inputIds" },
                    { "outputIds" },
                    { "poseIds" },
                    { "restInputIds" },
                }
            }
        }
    );
}

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2018 Ingo Clemens, brave rabbit
// weightDriver is under the terms of the MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Ingo Clemens    www.braverabbit.com
// ---------------------------------------------------------------------


