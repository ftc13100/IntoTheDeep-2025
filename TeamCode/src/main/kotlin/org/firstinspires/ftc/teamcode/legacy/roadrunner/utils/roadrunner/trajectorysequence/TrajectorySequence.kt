package org.firstinspires.ftc.teamcode.legacy.roadrunner.utils.roadrunner.trajectorysequence

//class TrajectorySequence(sequenceList: List<SequenceSegment>) {
//    private val sequenceList: List<SequenceSegment>
//
//    init {
//        if (sequenceList.isEmpty()) throw EmptySequenceException()
//        this.sequenceList = Collections.unmodifiableList(sequenceList)
//    }
//
//    fun start(): Pose2d {
//        return sequenceList[0].startPose
//    }
//
//    fun end(): Pose2d {
//        return sequenceList[sequenceList.size - 1].endPose
//    }
//
//    fun duration(): Double {
//        var total = 0.0
//        for (segment in sequenceList) {
//            total += segment.duration
//        }
//        return total
//    }
//
//    operator fun get(i: Int): SequenceSegment {
//        return sequenceList[i]
//    }
//
//    fun size(): Int {
//        return sequenceList.size
//    }
//}