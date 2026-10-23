#include "VsimHUD.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "VsimPlayerController.h"
#include "VsimWorld.h"

namespace
{
	const FLinearColor Ink(0.93f, 0.95f, 0.97f);
	const FLinearColor Muted(0.6f, 0.64f, 0.7f);
	const FLinearColor Good(0.24f, 0.81f, 0.56f);
	const FLinearColor Bad(1.0f, 0.36f, 0.36f);
	const FLinearColor Back(0.03f, 0.04f, 0.05f, 0.65f);

	FString LapTime(float Seconds)
	{
		if (Seconds <= 0.0f)
		{
			return TEXT("-");
		}
		const int32 Minutes = FMath::FloorToInt(Seconds / 60.0f);
		return Minutes > 0 ? FString::Printf(TEXT("%d:%06.3f"), Minutes, Seconds - 60.0f * Minutes)
						   : FString::Printf(TEXT("%.3f"), Seconds);
	}
}

void AVsimHUD::Panel(float X, float Y, float W, float H)
{
	DrawRect(Back, X, Y, W, H);
}

void AVsimHUD::DrawHUD()
{
	Super::DrawHUD();
	AVsimPlayerController* PC = Cast<AVsimPlayerController>(GetOwningPlayerController());
	AVsimWorld* Vsim = PC ? PC->GetVsimWorld() : nullptr;
	if (!Canvas || !Vsim || !PC->IsHudVisible())
	{
		return;
	}
	UFont* Large = GEngine->GetLargeFont();
	UFont* Medium = GEngine->GetMediumFont();
	UFont* Small = GEngine->GetSmallFont();
	const float Scale = FMath::Max(1.0f, Canvas->ClipY / 1080.0f);
	const float W = Canvas->ClipX;
	const float H = Canvas->ClipY;
	const double Fps = Vsim->GetFrameMs() > 0.0 ? 1000.0 / Vsim->GetFrameMs() : 0.0;
	SmoothedFps += 0.05 * (Fps - SmoothedFps);

	if (!Vsim->IsConnected())
	{
		const FString Waiting = FString::Printf(
			TEXT("Waiting for vsim-core (realtime mode, shared memory '%s')..."), *Vsim->LinkName);
		Panel(W * 0.5f - 330 * Scale, H * 0.45f, 660 * Scale, 40 * Scale);
		DrawText(Waiting, Ink, W * 0.5f - 315 * Scale, H * 0.45f + 10 * Scale, Medium, Scale);
		return;
	}
	const vsim_link::VehicleState& S = Vsim->GetState();

	// Bottom centre: speed, pedals, steering, energy.
	const float DashW = 520 * Scale, DashH = 110 * Scale;
	const float DashX = W * 0.5f - DashW * 0.5f, DashY = H - DashH - 16 * Scale;
	Panel(DashX, DashY, DashW, DashH);
	DrawText(FString::Printf(TEXT("%3.0f"), S.speed * 3.6), Ink, DashX + 18 * Scale, DashY + 14 * Scale, Large, 2.6f * Scale);
	DrawText(TEXT("km/h"), Muted, DashX + 30 * Scale, DashY + 84 * Scale, Small, Scale);
	const float BarX = DashX + 150 * Scale, BarY = DashY + 16 * Scale, BarH = 78 * Scale, BarW = 14 * Scale;
	DrawRect(FLinearColor(1, 1, 1, 0.08f), BarX, BarY, BarW, BarH);
	DrawRect(Good, BarX, BarY + BarH * (1 - S.throttle), BarW, BarH * S.throttle);
	DrawRect(FLinearColor(1, 1, 1, 0.08f), BarX + 24 * Scale, BarY, BarW, BarH);
	DrawRect(Bad, BarX + 24 * Scale, BarY + BarH * (1 - S.brake), BarW, BarH * S.brake);
	const float TextX = DashX + 210 * Scale;
	DrawText(FString::Printf(TEXT("Steer %4.0f deg"), FMath::RadiansToDegrees(S.steering_wheel)), Ink, TextX, DashY + 14 * Scale, Medium, Scale);
	DrawText(FString::Printf(TEXT("Power %6.1f kW%s"), S.power_kw, S.power_kw < -0.5f ? TEXT("  regen") : TEXT("")), Ink, TextX, DashY + 38 * Scale, Medium, Scale);
	DrawText(FString::Printf(TEXT("SoC %5.1f %%   motor %3.0f C"), 100.0f * S.soc, S.motor_temp), Ink, TextX, DashY + 62 * Scale, Medium, Scale);
	DrawText(FString::Printf(TEXT("lat %+.2f g   long %+.2f g"), S.ay / 9.81, S.ax / 9.81), Muted, TextX, DashY + 86 * Scale, Small, Scale);

	// Top centre: lap.
	if (S.track_generation != 0)
	{
		const float LapW = 640 * Scale, LapX = W * 0.5f - LapW * 0.5f, LapY = 14 * Scale;
		Panel(LapX, LapY, LapW, 46 * Scale);
		const FString Lap = S.finished ? FString(TEXT("Finished")) : FString::Printf(TEXT("Lap %d"), S.lap + 1);
		DrawText(Lap, Ink, LapX + 14 * Scale, LapY + 10 * Scale, Medium, 1.2f * Scale);
		DrawText(LapTime(S.lap_time), Ink, LapX + 120 * Scale, LapY + 8 * Scale, Large, 1.1f * Scale);
		DrawText(FString::Printf(TEXT("last %s"), *LapTime(S.last_lap)), S.last_lap > 0 && !S.last_lap_valid ? Bad : Muted,
			LapX + 270 * Scale, LapY + 14 * Scale, Medium, Scale);
		DrawText(FString::Printf(TEXT("best %s"), *LapTime(S.best_lap)), Muted, LapX + 400 * Scale, LapY + 14 * Scale, Medium, Scale);
		DrawText(FString::Printf(TEXT("S%d"), S.sector + 1), Muted, LapX + 530 * Scale, LapY + 14 * Scale, Medium, Scale);
		if (S.off_track || !S.lap_valid)
		{
			DrawText(S.off_track ? TEXT("OFF TRACK") : TEXT("lap invalid"), Bad, LapX + 570 * Scale, LapY + 14 * Scale, Medium, Scale);
		}
	}

	// Top left: link and timing.
	const float InfoX = 14 * Scale, InfoY = 14 * Scale;
	Panel(InfoX, InfoY, 330 * Scale, 128 * Scale);
	const FString Scenario = UTF8_TO_TCHAR(S.info.scenario);
	DrawText(FString::Printf(TEXT("%s  (%s)"), *Scenario, *Vsim->GetTrackName()), Ink, InfoX + 10 * Scale, InfoY + 8 * Scale, Small, Scale);
	DrawText(FString::Printf(TEXT("camera %s   %.0f fps"), VsimCameraName(Vsim->GetCamera()), SmoothedFps), Muted, InfoX + 10 * Scale, InfoY + 30 * Scale, Small, Scale);
	DrawText(FString::Printf(TEXT("input -> frame ~%.0f ms (frame %.1f ms)"), Vsim->GetInputLatencyMs() + Vsim->GetFrameMs(), Vsim->GetFrameMs()),
		Vsim->GetInputLatencyMs() + Vsim->GetFrameMs() < 50.0 ? Good : Bad, InfoX + 10 * Scale, InfoY + 52 * Scale, Small, Scale);
	DrawText(FString::Printf(TEXT("sim %.1f s  x%.2f  %s"), S.sim_time, S.rtf, S.wheel_connected ? TEXT("wheel") : TEXT("keyboard")),
		Muted, InfoX + 10 * Scale, InfoY + 74 * Scale, Small, Scale);
	if (S.paused)
	{
		DrawText(TEXT("PAUSED"), Bad, InfoX + 10 * Scale, InfoY + 98 * Scale, Medium, Scale);
	}

	if (PC->IsHelpVisible())
	{
		const float HelpX = W - 300 * Scale, HelpY = 14 * Scale;
		Panel(HelpX, HelpY, 286 * Scale, 100 * Scale);
		DrawText(TEXT("arrows/WASD drive (or the wheel)"), Muted, HelpX + 10 * Scale, HelpY + 8 * Scale, Small, Scale);
		DrawText(TEXT("C camera   R reset   Space pause"), Muted, HelpX + 10 * Scale, HelpY + 30 * Scale, Small, Scale);
		DrawText(TEXT("H HUD   F1 this help   Esc quit"), Muted, HelpX + 10 * Scale, HelpY + 52 * Scale, Small, Scale);
		DrawText(FString::Printf(TEXT("N sound (%s)   vsim viewer"), Vsim->IsSoundOn() ? TEXT("on") : TEXT("off")), Muted,
			HelpX + 10 * Scale, HelpY + 74 * Scale, Small, Scale);
	}
}
