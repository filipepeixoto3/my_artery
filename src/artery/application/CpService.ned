

//import artery.application.ItsG5Service;

//
// CollectivePerceptionMockService generates mock CPMs, i.e. without ASN.1 encoding.
// These mock CPMs include the list of detected objects and field-of-view containers
// of the equipped sensors.
//
package artery.application;

simple CpService like ItsG5Service
{
    parameters:
        @signal[CpmReceived](type=CpObject);
        @signal[CpmSent](type=CpObject);

        @statistic[reception](source=CpmReceived;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CPM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CPM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CPM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);

	int port = default(7777);  // zmq port
}
