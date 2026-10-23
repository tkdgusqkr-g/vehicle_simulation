// Viewer side of the vsim shared-memory link (vsim_link.h). Same logic as
// vsim::StateLinkReader in the simulator (core/src/link/state_link.cpp).
#pragma once

#include "CoreMinimal.h"
#include "vsim_link.h"

class VSIMLINK_API FVsimLinkReader
{
public:
	explicit FVsimLinkReader(const FString& InName);
	~FVsimLinkReader();
	FVsimLinkReader(const FVsimLinkReader&) = delete;
	FVsimLinkReader& operator=(const FVsimLinkReader&) = delete;

	/** Maps the state region if a live simulator publishes it; reopens after a restart. */
	bool Connect();
	/** A consistent copy of the newest state. */
	bool Read(vsim_link::VehicleState& Out) const;
	/**
	 * The track (with scenery) / car mesh the simulator publishes now.
	 * OutGeneration is its generation (0: none). Returns nullptr while the
	 * region cannot be mapped yet.
	 */
	const vsim_link::MeshRegion* GetTrack(uint64& OutGeneration);
	const vsim_link::MeshRegion* GetCar(uint64& OutGeneration);

	void SendKeys(uint32 Keys);
	void SendCommand(vsim_link::Command Command);
	/** Heartbeat plus what the viewer measured, shown by the simulator. */
	void ReportViewer(double LatencyMs, double FrameMs);

private:
	struct FMapping
	{
		void* Data = nullptr;
		SIZE_T Size = 0;
		void* Handle = nullptr;

		bool Open(const FString& Name);
		void Close();
	};

	vsim_link::StateRegion* Region() const { return static_cast<vsim_link::StateRegion*>(State.Data); }
	bool Usable() const;

	FString Name;
	const vsim_link::MeshRegion* Follow(uint64 Generation, const char (&RegionName)[64], FMapping& Mapping,
		uint64& MappedGeneration);

	FMapping State;
	FMapping Track;
	FMapping Car;
	uint64 TrackGeneration = 0;
	uint64 CarGeneration = 0;
	double NextAttempt = 0.0;
};
