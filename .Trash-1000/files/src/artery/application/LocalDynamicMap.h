#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
//add by lip
#include "artery/application/CpObject.h"
#include <vanetza/asn1/cpm.hpp>
//end add by lip
#include <cstdint>
#include <functional>
#include <map>
#include <memory>

namespace artery
{

class Timer;

class LocalDynamicMap
{
public:
    using StationID = uint32_t;
    using Cam = vanetza::asn1::Cam;
    using CamPredicate = std::function<bool(const Cam&)>;
    //add by lip
    using Cpm = vanetza::asn1::Cpm;
    using CpmPredicate = std::function<bool(const Cpm&)>;
    //end add by lip

    class AwarenessEntry
    {
    public:
        AwarenessEntry(const CaObject&, omnetpp::SimTime);
        AwarenessEntry(AwarenessEntry&&) = default;
        AwarenessEntry& operator=(AwarenessEntry&&) = default;

        omnetpp::SimTime expiry() const { return mExpiry; }
        const Cam& cam() const { return mObject.asn1(); }
        std::shared_ptr<const Cam> camPtr() const { return mObject.shared_ptr(); }

    private:
        omnetpp::SimTime mExpiry;
        CaObject mObject;
    };
    //add by lip
    class PerceptionEntry
    {
    public:
        PerceptionEntry(const CpObject&, omnetpp::SimTime);
        PerceptionEntry(PerceptionEntry&&) = default;
        PerceptionEntry& operator=(PerceptionEntry&&) = default;

        omnetpp::SimTime expiry() const { return m2Expiry; }
        const Cpm& cpm() const { return m2Object.asn1(); }
        std::shared_ptr<const Cpm> cpmPtr() const { return m2Object.shared_ptr(); }

    private:
        omnetpp::SimTime m2Expiry;
        CpObject m2Object;
    };
    //end add by lip 

    using AwarenessEntries = std::map<StationID, AwarenessEntry>;
    //add by lip
    using PerceptionEntries = std::map<StationID, PerceptionEntry>;
    //end ad by lip

    LocalDynamicMap(const Timer&);
    void updateAwareness(const CaObject&);
    void dropExpired();
    unsigned count(const CamPredicate&) const;
    std::shared_ptr<const Cam> getCam(StationID) const;
    const AwarenessEntries& allEntries() const { return mCaMessages; }
    //add by lip
    void updatePerception(const CpObject&);
    unsigned count2(const CpmPredicate&) const;
    std::shared_ptr<const Cpm> getCpm(StationID) const;
    const PerceptionEntries& allEntries2() const { return mCpMessages; }
    //end add by lip
    
private:
    const Timer& mTimer;
    AwarenessEntries mCaMessages;
    //add by lip
    PerceptionEntries mCpMessages;
    //end add by lip
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

