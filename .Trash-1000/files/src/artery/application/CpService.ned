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
	
	string host = default("192.168.31.248");  // zmq port
	int port = default(5555);  // zmq port

    // Multi-Object Tracking and Kalman Filter setup

    double dt = default(0.05); // :param dt: sampling time (time for 1 cycle)
    double u_x = default(0.0); // :param u_x: acceleration in x-direction
    double u_y = default(0.0); // :param u_y: acceleration in y-direction
    double std_acc = default(0.0); // :param std_acc: process noise magnitude
    double x_std_meas = default(0.3); // :param x_std_meas: standard deviation of the measurement in x-direction
    y_std_meas = default(0.3); // :param y_std_meas: standard deviation of the measurement in y-direction
}
