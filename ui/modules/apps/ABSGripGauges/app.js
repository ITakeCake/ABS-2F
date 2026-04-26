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
      $scope.speedData = {
        airspeed: '--',
        fusedSpeed: '--',
        plausibleSpeed: '--',
        virtualAirspeed: '--',
        fCircVal: '1.00',
        snapUpCount: 0,
        snapDnCount: 0,
        fusedActive: false
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
