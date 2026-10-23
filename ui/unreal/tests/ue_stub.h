// Minimal stand-ins for the Unreal Engine API that the VsimLink plugin uses,
// for check_plugin_syntax.py: catches syntax and type errors without the
// engine. Not the real API: signatures are simplified to what the plugin calls.
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <initializer_list>
#include <type_traits>
#include <cstdarg>

#define PLATFORM_WINDOWS 0
#define VSIMLINK_API
#define UCLASS(...)
#define UENUM(...)
#define UPROPERTY(...)
#define UFUNCTION(...)
#define GENERATED_BODY() public: typedef StubSelf Super; static class UClass* StaticClass(); private:
#define TEXT(x) L##x
#define UTF8_TO_TCHAR(x) StubConv(x)
#define TCHAR_TO_UTF8(x) "stub"
#define UE_LOG(...) do {} while (0)
#define IMPLEMENT_MODULE(a, b)
#define IMPLEMENT_PRIMARY_GAME_MODULE(a, b, c)
using TCHAR = wchar_t;
using ANSICHAR = char;
using uint8 = std::uint8_t; using uint32 = std::uint32_t; using int32 = std::int32_t; using uint64 = std::uint64_t; using int64 = std::int64_t;
using SIZE_T = std::size_t;
class UClass {};
class FString {
 public:
  FString() = default;
  FString(const TCHAR* s) : s_(s) {}
  FString(const std::wstring& s) : s_(s) {}
  const TCHAR* operator*() const { return s_.c_str(); }
  FString operator+(const FString& o) const { return FString(s_ + o.s_); }
  bool StartsWith(const TCHAR* p) const { return s_.rfind(p, 0) == 0; }
  bool operator==(const TCHAR* o) const { return s_ == o; }
  bool operator!=(const TCHAR* o) const { return s_ != o; }
  void Reset() { s_.clear(); }
  template <typename... A> static FString Printf(const TCHAR*, A...) { return FString(); }
 private:
  std::wstring s_;
};
inline FString StubConv(const char*) { return FString(); }
struct FName { FName(const TCHAR*) {} };
template <typename T> class TArray {
 public:
  TArray() = default;
  void Add(const T& v) { v_.push_back(v); }
  void Reserve(std::size_t n) { v_.reserve(n); }
  void Reset() { v_.clear(); }
  int32 Num() const { return int32(v_.size()); }
  T& operator[](int32 i) { return v_[std::size_t(i)]; }
  const T& operator[](int32 i) const { return v_[std::size_t(i)]; }
  auto begin() { return v_.begin(); } auto end() { return v_.end(); }
  auto begin() const { return v_.begin(); } auto end() const { return v_.end(); }
 private:
  std::vector<T> v_;
};
template <typename T> class TObjectPtr {
 public:
  TObjectPtr() = default;
  TObjectPtr(T* p) : p_(p) {}
  template <typename U, typename = std::enable_if_t<std::is_convertible_v<U*, T*>>> TObjectPtr(U* p) : p_(p) {}
  T* operator->() const { return p_; }
  operator T*() const { return p_; }
  T* Get() const { return p_; }
  explicit operator bool() const { return p_ != nullptr; }
 private:
  T* p_ = nullptr;
};
template <typename T> class TWeakObjectPtr {
 public:
  TWeakObjectPtr& operator=(T* p) { p_ = p; return *this; }
  bool IsValid() const { return p_ != nullptr; }
  T* Get() const { return p_; }
 private:
  T* p_ = nullptr;
};
template <typename T> struct TUniquePtr : std::unique_ptr<T> { using std::unique_ptr<T>::unique_ptr; TUniquePtr(std::unique_ptr<T>&& p) : std::unique_ptr<T>(std::move(p)) {} void Reset() { this->reset(); } T* Get() const { return this->get(); } };
template <typename T, typename... A> TUniquePtr<T> MakeUnique(A&&... a) { return std::make_unique<T>(std::forward<A>(a)...); }
struct FVector {
  double X = 0, Y = 0, Z = 0;
  FVector() = default;
  FVector(double x, double y, double z) : X(x), Y(y), Z(z) {}
  explicit FVector(double v) : X(v), Y(v), Z(v) {}
  FVector operator+(const FVector& o) const { return {X + o.X, Y + o.Y, Z + o.Z}; }
  FVector operator-(const FVector& o) const { return {X - o.X, Y - o.Y, Z - o.Z}; }
  FVector operator*(double s) const { return {X * s, Y * s, Z * s}; }
  FVector operator/(double s) const { return {X / s, Y / s, Z / s}; }
  FVector GetSafeNormal2D() const { return *this; }
  FVector GetSafeNormal() const { return *this; }
  double Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
  FVector& operator=(const FVector&) = default;
  static const FVector XAxisVector;
  struct FRotator Rotation() const;
  static double Dist(const FVector&, const FVector&) { return 0; }
  static double DistSquared(const FVector&, const FVector&) { return 0; }
  static const FVector ZeroVector;
};
struct FVector2D { FVector2D(double, double) {} };
struct FRotator { double Pitch = 0, Yaw = 0, Roll = 0; FRotator() = default; FRotator(double p, double y, double r) : Pitch(p), Yaw(y), Roll(r) {} FVector RotateVector(const FVector& v) const { return v; } static const FRotator ZeroRotator; };
struct FQuat { FQuat(const FVector&, double) {} };
struct FMatrix { FRotator Rotator() const { return {}; } };
struct FRotationMatrix { static FMatrix MakeFromX(const FVector&) { return {}; } };
inline FRotator FVector::Rotation() const { return {}; }
struct FTransform { static const FTransform Identity; };
struct FColor { FColor(uint8, uint8, uint8, uint8 = 255) {} };
struct FLinearColor {
  float R = 0, G = 0, B = 0, A = 1;
  FLinearColor() = default;
  FLinearColor(float r, float g, float b, float a = 1.0f) : R(r), G(g), B(b), A(a) {}
  FColor ToFColor(bool) const { return FColor(0, 0, 0); }
  static FLinearColor FromSRGBColor(const FColor&) { return {}; }
};
struct FProcMeshTangent {};
struct FMath {
  static double RadiansToDegrees(double r) { return r; }
  template <typename T> static T Max(T a, T b) { return a > b ? a : b; }
  template <typename T> static T Min(T a, T b) { return a < b ? a : b; }
  template <typename T> static T Max3(T a, T b, T c) { return Max(a, Max(b, c)); }
  template <typename T> static T Clamp(T v, T lo, T hi) { return v < lo ? lo : v > hi ? hi : v; }
  static double Atan(double v) { return std::atan(v); }
  static int32 FloorToInt(float v) { return int32(std::floor(v)); }
};
struct FMemory {
  static void* Memcpy(void* d, const void* s, SIZE_T n) { return std::memcpy(d, s, n); }
  static int32 Memcmp(const void* a, const void* b, SIZE_T n) { return std::memcmp(a, b, n); }
};
struct FCStringAnsi { static int32 Strcmp(const char* a, const char* b) { return std::strcmp(a, b); } };
struct FPlatformTime { static double Seconds() { return 0; } };

class UObject { public: static UClass* StaticClass() { return nullptr; } virtual ~UObject() = default; class UWorld* GetWorld() const { return nullptr; } };
template <typename T> T* NewObject(UObject*) { return new T(); }
template <typename T> T* LoadObject(UObject*, const TCHAR*) { return nullptr; }
template <typename T, typename U> T* Cast(U* p) { return dynamic_cast<T*>(p); }
class UStaticMesh : public UObject {};
class UMaterialInterface : public UObject {};
class UMaterialInstanceDynamic : public UMaterialInterface {
 public:
  static UMaterialInstanceDynamic* Create(UMaterialInterface*, UObject*) { return nullptr; }
  void SetVectorParameterValue(FName, const FLinearColor&) {}
  void SetScalarParameterValue(FName, float) {}
};
class UFont : public UObject {};
namespace EComponentMobility { enum Type { Static, Stationary, Movable }; }
namespace ECollisionEnabled { enum Type { NoCollision, QueryAndPhysics }; }
class UActorComponent : public UObject {
 public:
  void RegisterComponent() {}
  void DestroyComponent() {}
  void SetActive(bool) {}
  void SetAutoActivate(bool) {}
};
class USceneComponent : public UActorComponent {
 public:
  void SetMobility(EComponentMobility::Type) {}
  void SetupAttachment(USceneComponent*, FName = FName(L"")) {}
  void SetRelativeLocation(const FVector&) {}
  void SetRelativeRotation(const FRotator&) {}
  void SetRelativeRotation(const FQuat&) {}
  void SetRelativeScale3D(const FVector&) {}
  void SetVisibility(bool) {}
};
class UPrimitiveComponent : public USceneComponent {
 public:
  void SetCollisionEnabled(ECollisionEnabled::Type) {}
  void SetCastShadow(bool) {}
  void SetMaterial(int32, UMaterialInterface*) {}
};
class UStaticMeshComponent : public UPrimitiveComponent { public: void SetStaticMesh(UStaticMesh*) {} };
class UProceduralMeshComponent : public UPrimitiveComponent {
 public:
  bool bUseAsyncCooking = false;
  void ClearAllMeshSections() {}
  void CreateMeshSection_LinearColor(int32, const TArray<FVector>&, const TArray<int32>&, const TArray<FVector>&,
                                     const TArray<FVector2D>&, const TArray<FLinearColor>&, const TArray<FProcMeshTangent>&, bool) {}
};
class UCameraComponent : public USceneComponent { public: void SetFieldOfView(float) {} };
class USpringArmComponent : public USceneComponent {
 public:
  static const FName SocketName;
  float TargetArmLength = 0, CameraLagSpeed = 0, CameraRotationLagSpeed = 0;
  bool bDoCollisionTest = true, bEnableCameraLag = false, bEnableCameraRotationLag = false, bInheritPitch = true, bInheritRoll = true;
};
class ULightComponent : public USceneComponent { public: void SetIntensity(float) {} void SetCastShadows(bool) {} };
class UDirectionalLightComponent : public ULightComponent { public: void SetAtmosphereSunLight(bool) {} };
enum ESkyLightSourceType { SLS_CapturedScene, SLS_SpecifiedCubemap };
class USkyLightComponent : public ULightComponent { public: bool bRealTimeCapture = false; ESkyLightSourceType SourceType = SLS_CapturedScene; };
class USkyAtmosphereComponent : public USceneComponent {};
class UExponentialHeightFogComponent : public USceneComponent { public: void SetFogDensity(float) {} };
class UVolumetricCloudComponent : public USceneComponent {};
enum ETickingGroup { TG_PrePhysics, TG_PostUpdateWork };
struct FActorTickFunction { bool bCanEverTick = false; ETickingGroup TickGroup = TG_PrePhysics; };
namespace EEndPlayReason { enum Type { Destroyed, Quit }; }
enum class ESpawnActorCollisionHandlingMethod { AlwaysSpawn };
struct FActorSpawnParameters { class AActor* Owner = nullptr; ESpawnActorCollisionHandlingMethod SpawnCollisionHandlingOverride{}; };
class AActor : public UObject {
 public:
  using StubSelf = AActor;
  FActorTickFunction PrimaryActorTick;
  USceneComponent* RootComponent = nullptr;
  template <typename T> T* CreateDefaultSubobject(const TCHAR*) { return new T(); }
  virtual void BeginPlay() {}
  virtual void EndPlay(const EEndPlayReason::Type) {}
  virtual void Tick(float) {}
  FVector GetActorLocation() const { return {}; }
  bool SetActorLocationAndRotation(const FVector&, const FRotator&) { return true; }
};
class UWorld : public UObject {
 public:
  template <typename T> T* SpawnActor(UClass*, const FTransform&, const FActorSpawnParameters& = {}) { return nullptr; }
  class APlayerController* GetFirstPlayerController() const { return nullptr; }
};
class ACameraActor : public AActor { public: UCameraComponent* GetCameraComponent() const { return nullptr; } };
struct FKey {};
struct EKeys { static const FKey Up, Down, Left, Right, W, A, S, D, C, R, P, H, F1, SpaceBar, Escape; };
struct FInputModeGameOnly {};
class AController : public AActor {};
class APlayerController : public AController {
 public:
  using StubSelf = APlayerController;
  bool bAutoManageActiveCameraTarget = true;
  bool bShowMouseCursor = true;
  virtual void PlayerTick(float) {}
  bool IsInputKeyDown(const FKey&) const { return false; }
  bool WasInputKeyJustPressed(const FKey&) const { return false; }
  void SetInputMode(const FInputModeGameOnly&) {}
  void ConsoleCommand(const FString&, bool = true) {}
  AActor* GetViewTarget() const { return nullptr; }
  void SetViewTarget(AActor*) {}
};
class UCanvas : public UObject { public: float ClipX = 0, ClipY = 0; };
class AHUD : public AActor {
 public:
  using StubSelf = AHUD;
  UCanvas* Canvas = nullptr;
  virtual void DrawHUD() {}
  APlayerController* GetOwningPlayerController() const { return nullptr; }
  void DrawText(const FString&, FLinearColor, float, float, UFont* = nullptr, float = 1.f, bool = false) {}
  void DrawRect(FLinearColor, float, float, float, float) {}
};
class UEngine { public: UFont* GetLargeFont() { return nullptr; } UFont* GetMediumFont() { return nullptr; } UFont* GetSmallFont() { return nullptr; } };
extern UEngine* GEngine;
class AGameModeBase : public AActor { public: using StubSelf = AGameModeBase; UClass* PlayerControllerClass = nullptr; UClass* HUDClass = nullptr; UClass* DefaultPawnClass = nullptr; };
template <typename T> class TActorIterator {
 public:
  explicit TActorIterator(UWorld*) {}
  explicit operator bool() const { return false; }
  TActorIterator& operator++() { return *this; }
  T* operator*() const { return nullptr; }
};
namespace ConstructorHelpers {
template <typename T> struct FObjectFinder { explicit FObjectFinder(const TCHAR*) {} T* Object = nullptr; };
}
class FDefaultModuleImpl {};
class FDefaultGameModuleImpl {};
