angular.module('beamng.apps')
.directive('absGripGauges', [function () {
  return {
    templateUrl: '/ui/modules/apps/ABSGripGauges/app.html',
    replace: true,
    restrict: 'EA',
    controller: ['$scope', function ($scope) {
      $scope.gripData = {
        FL: { surfaceMu: '--', slipMu: '--' },
        FR: { surfaceMu: '--', slipMu: '--' },
        RL: { surfaceMu: '--', slipMu: '--' },
        RR: { surfaceMu: '--', slipMu: '--' }
      };

      setTimeout(function() {
        if (window.bngApi && window.bngApi.activeObjectLua) {
          window.bngApi.activeObjectLua('extensions.load("absTelemetryLogger")');
        }
      }, 500);

      $scope.speedData = {
        airspeed: '--',
        fusedSpeed: '--',
        maxWs: '--',
        seekTarget: '--',
        seekScore: '--',
        plausibleSpeed: '--',
        virtualAirspeed: '--',
        fCircVal: '1.00',
        snapUpCount: 0,
        snapDnCount: 0,
        snapUpRejCount: 0,
        imuSpeed: '--',
        imuClampCount: 0,
        phantomVetoCount: 0,
        stuckResyncCount: 0,
        vLat: '--',
        yaw2d: '--',
        probeCount: 0,
        probeFastCount: 0,
        probeDriftCount: 0,
        ndwCount: 0,
        fusedActive: false,
        lockupGuardCount: 0,
        nextDEstimate: '--',
        slipStepdownCount: 0,
        surfaceChangeCount: 0,
        reanchorCount: 0,
        reanchorDelta: '--',
        onsetSnaps: 0,
        flightEvents: 0,
        flightDumps: 0,
        flightOpen: false,
        support: '--'
      };

      $scope.$on('updateABSGrip', function(event, data) {
        $scope.$evalAsync(function() {
          if (data && data.FL) {
            $scope.gripData = {
              FL: data.FL,
              FR: data.FR,
              RL: data.RL,
              RR: data.RR
            };
          }
          if (data && data.speeds) {
            $scope.speedData = data.speeds;
          }
        });
      });
    }]
  };
}]);
