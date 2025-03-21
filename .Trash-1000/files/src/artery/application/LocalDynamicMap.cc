#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include <omnetpp/csimulation.h>
#include <cassert>
#include <algorithm>

namespace artery
{

LocalDynamicMap::LocalDynamicMap(const Timer& timer) :
    mTimer(timer)
{
}

void LocalDynamicMap::updateAwareness(const CaObject& obj)
{
    const vanetza::asn1::Cam& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->cam.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CAM is out of bounds";
        return;
    }

    AwarenessEntry entry(obj, expiry);
    auto found = mCaMessages.find(msg->header.stationID);
    if (found != mCaMessages.end()) {
        found->second = std::move(entry);
    } else {
        mCaMessages.emplace(msg->header.stationID, std::move(entry));
    }
}

//add by lip
void LocalDynamicMap::updatePerception(const CpObject& obj)
{
    const vanetza::asn1::Cpm& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->cpm.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CPM is out of bounds";
        return;
    }

    PerceptionEntry entry(obj, expiry);
    auto found = mCpMessages.find(msg->header.stationID);
    if (found != mCpMessages.end()) {
        found->second = std::move(entry);
    } else {
        mCpMessages.emplace(msg->header.stationID, std::move(entry));
    }
}
//end add by lip

void LocalDynamicMap::dropExpired()
{
    const auto now = omnetpp::simTime();
    for (auto it = mCaMessages.begin(); it != mCaMessages.end();) {
        if (it->second.expiry() < now) {
            it = mCaMessages.erase(it);
        } else {
            ++it;
        }
    }
    //add by lip
    for (auto it = mCpMessages.begin(); it != mCpMessages.end();) {
        if (it->second.expiry() < now) {
            it = mCpMessages.erase(it);
        } else {
            ++it;
        }
    }
    //end add by lip
}

unsigned LocalDynamicMap::count(const CamPredicate& predicate) const
{
    return std::count_if(mCaMessages.begin(), mCaMessages.end(),
            [&predicate](const AwarenessEntries::value_type& map_entry) {
                return predicate(map_entry.second.cam());
            });
}

//add by lip
unsigned LocalDynamicMap::count2(const CpmPredicate& predicate) const
{
    return std::count_if(mCpMessages.begin(), mCpMessages.end(),
            [&predicate](const PerceptionEntries::value_type& map_entry) {
                return predicate(map_entry.second.cpm());
            });
}

std::shared_ptr<const LocalDynamicMap::Cpm> LocalDynamicMap::getCpm(StationID stationId) const
{
    auto cpm = mCpMessages.find(stationId);
    if (cpm != mCpMessages.end()) {
        return cpm->second.cpmPtr();
    }

    return nullptr;
}

//end add by lip

std::shared_ptr<const LocalDynamicMap::Cam> LocalDynamicMap::getCam(StationID stationId) const
{
    auto cam = mCaMessages.find(stationId);
    if (cam != mCaMessages.end()) {
        return cam->second.camPtr();
    }

    return nullptr;
}

LocalDynamicMap::AwarenessEntry::AwarenessEntry(const CaObject& obj, omnetpp::SimTime t) :
    mExpiry(t), mObject(obj)
{
}

//add by lip
LocalDynamicMap::PerceptionEntry::PerceptionEntry(const CpObject& obj, omnetpp::SimTime t) :
    m2Expiry(t), m2Object(obj)
{
}
//end add by lip

} // namespace artery
