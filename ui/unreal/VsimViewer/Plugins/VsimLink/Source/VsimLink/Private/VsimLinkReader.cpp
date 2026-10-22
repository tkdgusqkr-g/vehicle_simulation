#include "VsimLinkReader.h"

#include "HAL/PlatformTime.h"

#if PLATFORM_WINDOWS
// The engine's wrapper undoes windows.h macros such as DrawText afterwards.
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/WindowsHWrapper.h"
#include "Windows/HideWindowsPlatformTypes.h"
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

FVsimLinkReader::FVsimLinkReader(const FString& InName) : Name(InName) {}

FVsimLinkReader::~FVsimLinkReader()
{
	Track.Close();
	State.Close();
}

bool FVsimLinkReader::FMapping::Open(const FString& ObjectName)
{
	Close();
#if PLATFORM_WINDOWS
	const FString Full = FString(TEXT("Local\\")) + ObjectName;
	HANDLE Mapping = OpenFileMappingW(FILE_MAP_ALL_ACCESS, 0, *Full);
	if (!Mapping)
	{
		return false;
	}
	void* View = MapViewOfFile(Mapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	if (!View)
	{
		CloseHandle(Mapping);
		return false;
	}
	MEMORY_BASIC_INFORMATION Info = {};
	VirtualQuery(View, &Info, sizeof(Info));
	Data = View;
	Size = Info.RegionSize;
	Handle = Mapping;
	return true;
#else
	const FString Full = FString(TEXT("/")) + ObjectName;
	const int Fd = shm_open(TCHAR_TO_UTF8(*Full), O_RDWR, 0600);
	if (Fd < 0)
	{
		return false;
	}
	struct stat Info = {};
	if (fstat(Fd, &Info) != 0 || Info.st_size <= 0)
	{
		close(Fd);
		return false;
	}
	void* View = mmap(nullptr, static_cast<size_t>(Info.st_size), PROT_READ | PROT_WRITE, MAP_SHARED, Fd, 0);
	close(Fd);
	if (View == MAP_FAILED)
	{
		return false;
	}
	Data = View;
	Size = static_cast<SIZE_T>(Info.st_size);
	return true;
#endif
}

void FVsimLinkReader::FMapping::Close()
{
#if PLATFORM_WINDOWS
	if (Data)
	{
		UnmapViewOfFile(Data);
	}
	if (Handle)
	{
		CloseHandle(static_cast<HANDLE>(Handle));
	}
#else
	if (Data)
	{
		munmap(Data, Size);
	}
#endif
	Data = nullptr;
	Handle = nullptr;
	Size = 0;
}

bool FVsimLinkReader::Usable() const
{
	return State.Data && State.Size >= sizeof(vsim_link::StateRegion) && vsim_link::region_valid(*Region()) &&
		vsim_link::publisher_alive(*Region(), vsim_link::now_ns());
}

bool FVsimLinkReader::Connect()
{
	if (Usable())
	{
		return true;
	}
	// Retry twice a second, not every frame.
	const double Now = FPlatformTime::Seconds();
	if (Now < NextAttempt)
	{
		return false;
	}
	NextAttempt = Now + 0.5;
	Track.Close();
	TrackGeneration = 0;
	State.Open(Name);
	return Usable();
}

bool FVsimLinkReader::Read(vsim_link::VehicleState& Out) const
{
	return State.Data && vsim_link::read_state(*Region(), Out);
}

const vsim_link::TrackRegion* FVsimLinkReader::GetTrack(uint64& OutGeneration)
{
	OutGeneration = 0;
	if (!State.Data)
	{
		return nullptr;
	}
	const vsim_link::StateRegion& R = *Region();
	const uint64 Generation = R.track_generation.load(std::memory_order_acquire);
	OutGeneration = Generation;
	if (Generation == 0)
	{
		Track.Close();
		TrackGeneration = 0;
		return nullptr;
	}
	if (Generation != TrackGeneration)
	{
		ANSICHAR RegionName[sizeof(R.track_region) + 1] = {};
		FMemory::Memcpy(RegionName, R.track_region, sizeof(R.track_region));
		TrackGeneration = Track.Open(FString(UTF8_TO_TCHAR(RegionName))) ? Generation : 0;
	}
	if (!Track.Data || Track.Size < sizeof(vsim_link::TrackRegion))
	{
		return nullptr;
	}
	const auto* Mesh = static_cast<const vsim_link::TrackRegion*>(Track.Data);
	const bool bValid = Mesh->magic == vsim_link::kTrackMagic && Mesh->version == vsim_link::kVersion &&
		Mesh->size <= Track.Size && Mesh->generation == Generation;
	return bValid ? Mesh : nullptr;
}

void FVsimLinkReader::SendKeys(uint32 Keys)
{
	if (State.Data)
	{
		vsim_link::send_keys(*Region(), Keys);
	}
}

void FVsimLinkReader::SendCommand(vsim_link::Command Command)
{
	if (State.Data)
	{
		vsim_link::send_command(*Region(), Command);
	}
}

void FVsimLinkReader::ReportViewer(double LatencyMs, double FrameMs)
{
	if (State.Data)
	{
		vsim_link::ViewerChannel& Viewer = Region()->viewer;
		Viewer.latency_us.store(static_cast<int32>(LatencyMs * 1000.0), std::memory_order_relaxed);
		Viewer.frame_us.store(static_cast<int32>(FrameMs * 1000.0), std::memory_order_relaxed);
		Viewer.heartbeat_ns.store(vsim_link::now_ns(), std::memory_order_relaxed);
	}
}
