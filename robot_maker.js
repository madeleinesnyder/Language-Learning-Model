var fps = 60;
var mspf = 1000/fps;
var lastTime = 0;
window.requestAnimationFrame = function(callback, element) {
    var currTime = new Date().getTime();
    var timeToCall = Math.max(0, mspf/2 - (currTime - lastTime));  //run twice as fast...
    var id = window.setTimeout(function() { callback(currTime + timeToCall); },
      timeToCall);
    lastTime = currTime + timeToCall;
    return id;
};
window.cancelAnimationFrame = function(id) {
    clearTimeout(id);
};

var requestId;

function stopAnim() {
  if (requestId) {
    window.cancelAnimationFrame(requestId);
    requestId = undefined;
  }
}

var SCALE = 30; // 1 meter = 30 pixels
worldWidth = 350;
worldHeight = 500;

var  b2World = Box2D.Dynamics.b2World,
     b2Vec2 = Box2D.Common.Math.b2Vec2,
     b2AABB = Box2D.Collision.b2AABB,
     b2BodyDef = Box2D.Dynamics.b2BodyDef,
     b2Body = Box2D.Dynamics.b2Body,
     b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
     b2Fixture = Box2D.Dynamics.b2Fixture,
     b2MassData = Box2D.Collision.Shapes.b2MassData,
     b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
     b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
     b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
     b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef;

var world = new b2World(
   new b2Vec2(0, 10), //gravity
   true               //allow sleep
);

// same fixture definition for all objects
var fixDef = new b2FixtureDef;
fixDef.density = 1.0;
fixDef.friction = 0.2;
fixDef.restitution = 0.1;

var bodyDef = new b2BodyDef;
bodyDef.angle = 0;

var listToArray = function(list, recurse) {
    if (recurse) {
        return list.slice(0, -1).map(function (x) {return Array.isArray(x) ? listToArray(x) : x});
    } else {
        return list.slice(0, -1);
    }
};

var arrayToList = function(arr, mutate) {
    if (mutate) {
        arr.push(null); 
    } else {
        arr = arr.concat(null);
    }
    return arr;
};


// function listToArray(list, recurse) {
//  var array = [];
//  while (list.length > 0) {
//      var left = list[0];
//      array.push((Array.isArray(left) && recurse) ? listToArray(left) : left);
//      list = list[1];
//  }
//  return array;
// }

var the_empty_list = [];

// function arrayToList(arr) {
//  if (arr.length == 0) {
//      return the_empty_list;
//  } else {
//      return [arr[0], arrayToList(arr.slice(1))];
//  }
// }

function clearWorld() {
  var count = world.GetBodyCount();
  for (var i=0; i<count; i++) {
    var body = world.GetBodyList();
    world.DestroyBody(body);
  }
}

//take church world maker and apply it to the box2d world
function applyWorld(initialWorld) {
  var worldList = churchWorld_to_jsWorld(initialWorld);
  for (var i=0; i<worldList.length; i++) {
    var worldObj = worldList[i];
    var shapeProps = worldObj[0];
    var shape = shapeProps[0];
    var isStatic = shapeProps[1];
    var dims = shapeProps[2];
    var position = worldObj[1];
    if (worldObj.length > 2) {
      var velocity = worldObj[2];
    } else {
      var velocity = [0,0];
    }
    if (isStatic) {
      bodyDef.type = b2Body.b2_staticBody;
    } else {
      bodyDef.type = b2Body.b2_dynamicBody;
    }
    if (shape == "circle") {
      var r = dims[0] / SCALE;
      fixDef.shape = new b2CircleShape(r);
    } else if (shape == "rect") {
      var w = dims[0] / SCALE;
      var h = dims[1] / SCALE;
      fixDef.shape = new b2PolygonShape;
      fixDef.shape.SetAsBox(w, h);
    } else {
      console.log("error 0");
    }
    bodyDef.position.x = position[0] / SCALE;
    bodyDef.position.y = position[1] / SCALE;
    bodyDef.linearVelocity.x = velocity[0] / SCALE;
    bodyDef.linearVelocity.y = velocity[1] / SCALE;
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    /*if (shape == "rect") {
      console.log(myShape.GetBody().GetFixtureList().GetShape().GetVertices());
    }*/
  }
  return initialWorld;
}

function jsWorld_to_churchWorld(world) {
  return arrayToList(world.map(function(object) {
    return arrayToList(object.map(function(property) {
      return arrayToList(property.map(function(element) {
        if (Object.prototype.toString.call(element) === '[object Array]') {
          return arrayToList(element);
        } else {
          return element;
        }
      }));
    }));
  }));
}

function churchWorld_to_jsWorld(world) {
  return listToArray(world).map(function(object) {
    var object = listToArray(object).map(function(property) {
      return listToArray(property);
    });
    object[0][2] = listToArray(object[0][2]);
    return object;
  });
}

function churchWorld_from_bodyList(body) {
  var worldList = [];
  while (body) {
    var isStatic;
    if (body.GetType() == 2) {
      var isStatic = false;
    } else {
      var isStatic = true;
    }
    var shapeInt = body.GetFixtureList().GetType();
    var shape;
    var dims;
    if (shapeInt == 0) {
      shape = "circle";
      dims = [body.GetFixtureList().GetShape().GetRadius() * SCALE];
    } else {
      shape = "rect";
      vertices = body.GetFixtureList().GetShape().GetVertices();
      dims = [vertices[2].x * 2 * SCALE, vertices[2].y * 2 * SCALE];
    }
    var x = body.GetPosition().x * SCALE;
    var y = body.GetPosition().y * SCALE;
    worldList.push([ [shape, isStatic, dims], [x, y] ]);
    body = body.GetNext();
  }
  return jsWorld_to_churchWorld(worldList);
}

function getDynamicObjPositions(churchWorld) {
  var worldList = churchWorld_to_jsWorld(churchWorld);
  var positions = [];
  for (var i=0; i<worldList.length; i++) {
    var worldObj = worldList[i];
    var isStatic = worldObj[0][1];
    if (isStatic == false) {
      positions.push(worldObj[1]);
    }
  }
  return positions;
}

emptyWorld = arrayToList([]);

//add a circle at specified position (x and y are between 0 and 1) and radius. return world with circle added.
addCircle = function(churchWorld, x, y, r, isStatic, dx, dy) {
  var dx=dx || 0;
  var dy=dy || 0;
  var jsWorld = churchWorld_to_jsWorld(churchWorld);
  jsWorld.push( [ ["circle", isStatic, [r]],
                    [x, y],
                    [dx, dy] ] );
  return jsWorld_to_churchWorld(jsWorld);
}

addRect = function(churchWorld, x, y, w, h, isStatic, dx, dy) {
  var dx=dx || 0;
  var dy=dy || 0;
  var jsWorld = churchWorld_to_jsWorld(churchWorld);
  jsWorld.push( [ ["rect", isStatic, [w, h]],
                    [x, y],
                    [dx, dy] ] );
  return jsWorld_to_churchWorld(jsWorld);
}

//now in church
/*_plinkoWhichBin = function(finalWorld, ncol) {
  var positions = getDynamicObjPositions(finalWorld);
  var x = positions[0][0];
  return Math.round(x / (_worldWidth / ncol));
}*/

plinkoWorld = function(nrow, ncol) {
  var pegRadius = 3;
  var wallWidth = 5;
  var binHeight = 120;
  //ground
  var ground = [ [ "rect", true, [worldWidth, wallWidth] ],
                 [ worldWidth / 2, worldHeight ]];
  //pegs
  var pegs = [];
  var pegShapeProperties = ["circle", true, [pegRadius]];
  for (var r=0; r<nrow; r++) {
    for (var c=0; c<ncol; c++) {
      var xpos = worldWidth / (ncol + 1) * (c + 1);
      var ypos = (worldHeight - binHeight) / (nrow + 2) * (r+1);
      pegs.push([ pegShapeProperties,
                  [ xpos, ypos]]);}}
  //walls
  var wallShapeProperties = ["rect", true, [wallWidth, worldHeight]];
  function wall(xpos) {return [wallShapeProperties, [xpos, worldHeight / 2]];}
  var walls = [wall(0), wall(worldWidth)];
  //bins
  var bins = [];
  var binShapeProperties = ["rect", true, [wallWidth, binHeight]];
  var ypos = worldHeight - (binHeight/2);
  for (var c=0; c < ncol + 2; c++) {
    var xpos = _worldWidth / (ncol+1) * c;
    bins.push([binShapeProperties, [xpos, ypos]])}
  return jsWorld_to_churchWorld([ground].concat(pegs, walls, bins));
}

runPhysics = function(steps, initialWorld) {
  clearWorld();
  applyWorld(initialWorld);
  for (var s=0; s<steps; s++) {
    world.Step(
         1 / fps   //frame-rate
      ,  10       //velocity iterations
      ,  10       //position iterations
    );
  }
  return churchWorld_from_bodyList(world.GetBodyList());
}


animatePhysics = function(steps, initialWorld) {
  function simulate(canvas, steps, initializeStep) {
    clearWorld();
    applyWorld(initialWorld);
    //setup debug draw
    var debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(canvas[0].getContext("2d"));
    debugDraw.SetDrawScale(SCALE);
    debugDraw.SetFillAlpha(0.3);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    world.SetDebugDraw(debugDraw);

    function update(stepsSoFar) {
      stepsSoFar++;
      var currTime = new Date().getTime();
      requestId = requestAnimationFrame(function(time) {
        update(stepsSoFar);}
      );

      if (stepsSoFar < steps) {
        world.Step(
             1 / fps   //frame-rate
          ,  10       //velocity iterations
          ,  10       //position iterations
        );
      }
      
      world.DrawDebugData();
      world.ClearForces();
    };

    requestId = requestAnimationFrame(function() {update(0);});
  }

    sideEffects.push({
        type: 'function',
        data: function($div) {
            stopAnim(); //stop previous update thread..
            setTimeout(stopAnim, mspf); //make absolutely sure previous update thread is stopped
            var $physicsDiv = $("<div>").appendTo($div);
            $physicsDiv.append("<br/>");
            var $canvas = $("<canvas/>").appendTo($physicsDiv);
            $canvas.attr("width", worldWidth)
                .attr("style", "background-color:#333333;")
                .attr("height", worldHeight);
            $physicsDiv.append("<br/>");
            //var initializeStep = true;
            //simulate($canvas, 0, initializeStep);
            simulate($canvas, 0);
            //initializeStep = false;
            var $button = $("<button>Simulate</button>").appendTo($physicsDiv);
            $button.click(function() {
                //simulate($canvas, steps, initializeStep);
                
                stopAnim(); //stop previous update thread..
                simulate($canvas, steps);
                //initializeStep = true;
            });
            var $clearButton = $("<button>Delete Animation Window</button>")
            $clearButton.appendTo($physicsDiv);
            $clearButton.click(function() {
                var count = world.GetBodyCount();
                for (var i=0; i<count; i++) {
                    var body = world.GetBodyList();
                    world.DestroyBody(body);
                }
                $physicsDiv.remove();
            });
            return "";
        }
    });
}

var webppl = require("/usr/local/lib/node_modules/webppl/src/main.js");
var __runner__ = util.trampolineRunners.cli();
function printWebPPLValue(x) {
  if (dists.isDist(x) && x.print) {
    console.log(x.meta.name + ':');
    console.log(x.print());
  } else {
    console.log(x);
  }
};
function topK(s, x) {
  printWebPPLValue(x);
};
var main = (function (_globalCurrentAddress) {
    return function (p) {
        return function (runTrampoline) {
            return function (s, k, a) {
                runTrampoline(function () {
                    return p(s, k, a);
                });
            };
        };
    }(function (globalStore, _k0, _address0) {
        var _currentAddress = _address0;
        _addr.save(_globalCurrentAddress, _address0);
        var Bernoulli = function Bernoulli(globalStore, _k356, _address1, params) {
            var _currentAddress = _address1;
            _addr.save(_globalCurrentAddress, _address1);
            return function () {
                return _k356(globalStore, util.jsnew(dists.Bernoulli, params));
            };
        };
        var Gaussian = function Gaussian(globalStore, _k315, _address22, params) {
            var _currentAddress = _address22;
            _addr.save(_globalCurrentAddress, _address22);
            return function () {
                return _k315(globalStore, util.jsnew(dists.Gaussian, params));
            };
        };
        var gaussian = function gaussian(globalStore, _k312, _address23, arg0, arg1) {
            var _currentAddress = _address23;
            _addr.save(_globalCurrentAddress, _address23);
            var _k314 = function (globalStore, params) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                return function () {
                    return Gaussian(globalStore, function (globalStore, _result313) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        return function () {
                            return sample(globalStore, _k312, _address23.concat('_21'), _result313);
                        };
                    }, _address23.concat('_20'), params);
                };
            };
            return function () {
                return util.isObject(arg0) ? _k314(globalStore, arg0) : _k314(globalStore, {
                    mu: arg0,
                    sigma: arg1
                });
            };
        };
        var flip = function flip(globalStore, _k265, _address46, p) {
            var _currentAddress = _address46;
            _addr.save(_globalCurrentAddress, _address46);
            var _k268 = function (globalStore, _result267) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                var params = { p: _result267 };
                return function () {
                    return Bernoulli(globalStore, function (globalStore, _result266) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        return function () {
                            return sample(globalStore, _k265, _address46.concat('_43'), _result266);
                        };
                    }, _address46.concat('_42'), params);
                };
            };
            return function () {
                return ad.scalar.pneq(p, undefined) ? _k268(globalStore, p) : _k268(globalStore, 0.5);
            };
        };
        var dimA = function dimA(globalStore, _k68, _address154) {
            var _currentAddress = _address154;
            _addr.save(_globalCurrentAddress, _address154);
            return function () {
                return gaussian(globalStore, _k68, _address154.concat('_157'), 30, 10);
            };
        };
        var dimB = function dimB(globalStore, _k67, _address155) {
            var _currentAddress = _address155;
            _addr.save(_globalCurrentAddress, _address155);
            return function () {
                return gaussian(globalStore, _k67, _address155.concat('_158'), 30, 25);
            };
        };
        var dim_static = function dim_static(globalStore, _k65, _address157) {
            var _currentAddress = _address157;
            _addr.save(_globalCurrentAddress, _address157);
            return function () {
                return _k65(globalStore, 10);
            };
        };
        var dim_body1 = function dim_body1(globalStore, _k64, _address158) {
            var _currentAddress = _address158;
            _addr.save(_globalCurrentAddress, _address158);
            return function () {
                return _k64(globalStore, 20);
            };
        };
        var dim_body2 = function dim_body2(globalStore, _k63, _address159) {
            var _currentAddress = _address159;
            _addr.save(_globalCurrentAddress, _address159);
            return function () {
                return _k63(globalStore, 1);
            };
        };
        var dim_eye = function dim_eye(globalStore, _k62, _address160) {
            var _currentAddress = _address160;
            _addr.save(_globalCurrentAddress, _address160);
            return function () {
                return _k62(globalStore, 4);
            };
        };
        var shape = function shape(globalStore, _k61, _address161) {
            var _currentAddress = _address161;
            _addr.save(_globalCurrentAddress, _address161);
            return function () {
                return _k61(globalStore, 'rect');
            };
        };
        var xpos = function xpos(globalStore, _k60, _address162, x) {
            var _currentAddress = _address162;
            _addr.save(_globalCurrentAddress, _address162);
            return function () {
                return _k60(globalStore, x);
            };
        };
        var ypos = function ypos(globalStore, _k59, _address163, y) {
            var _currentAddress = _address163;
            _addr.save(_globalCurrentAddress, _address163);
            return function () {
                return _k59(globalStore, y);
            };
        };
        var eye = function eye(globalStore, _k58, _address164) {
            var _currentAddress = _address164;
            _addr.save(_globalCurrentAddress, _address164);
            return function () {
                return _k58(globalStore, 'circle');
            };
        };
        var ground = {
            shape: 'rect',
            static: !0,
            dims: [
                worldWidth,
                10
            ],
            x: ad.scalar.div(worldWidth, 2),
            y: worldHeight
        };
        var body = function body(globalStore, _k52, _address165) {
            var _currentAddress = _address165;
            _addr.save(_globalCurrentAddress, _address165);
            return function () {
                return shape(globalStore, function (globalStore, _result53) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return dim_body2(globalStore, function (globalStore, _result54) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            return function () {
                                return dim_body1(globalStore, function (globalStore, _result55) {
                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                    return function () {
                                        return xpos(globalStore, function (globalStore, _result56) {
                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                            return function () {
                                                return ypos(globalStore, function (globalStore, _result57) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    return function () {
                                                        return _k52(globalStore, {
                                                            shape: _result53,
                                                            static: !0,
                                                            dims: [
                                                                _result54,
                                                                _result55
                                                            ],
                                                            x: _result56,
                                                            y: _result57
                                                        });
                                                    };
                                                }, _address165.concat('_164'), ad.scalar.sub(worldHeight, 50));
                                            };
                                        }, _address165.concat('_163'), ad.scalar.div(worldWidth, 2));
                                    };
                                }, _address165.concat('_162'));
                            };
                        }, _address165.concat('_161'));
                    };
                }, _address165.concat('_160'));
            };
        };
        var eye1 = function eye1(globalStore, _k46, _address166) {
            var _currentAddress = _address166;
            _addr.save(_globalCurrentAddress, _address166);
            return function () {
                return eye(globalStore, function (globalStore, _result47) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return dim_eye(globalStore, function (globalStore, _result48) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            return function () {
                                return dim_eye(globalStore, function (globalStore, _result49) {
                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                    return function () {
                                        return xpos(globalStore, function (globalStore, _result50) {
                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                            return function () {
                                                return ypos(globalStore, function (globalStore, _result51) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    return function () {
                                                        return _k46(globalStore, {
                                                            shape: _result47,
                                                            static: !0,
                                                            dims: [
                                                                _result48,
                                                                _result49
                                                            ],
                                                            x: _result50,
                                                            y: _result51
                                                        });
                                                    };
                                                }, _address166.concat('_169'), ad.scalar.sub(worldHeight, 78.5));
                                            };
                                        }, _address166.concat('_168'), ad.scalar.sub(ad.scalar.div(worldWidth, 2), 7));
                                    };
                                }, _address166.concat('_167'));
                            };
                        }, _address166.concat('_166'));
                    };
                }, _address166.concat('_165'));
            };
        };
        var eye2 = function eye2(globalStore, _k40, _address167) {
            var _currentAddress = _address167;
            _addr.save(_globalCurrentAddress, _address167);
            return function () {
                return eye(globalStore, function (globalStore, _result41) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return dim_eye(globalStore, function (globalStore, _result42) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            return function () {
                                return dim_eye(globalStore, function (globalStore, _result43) {
                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                    return function () {
                                        return xpos(globalStore, function (globalStore, _result44) {
                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                            return function () {
                                                return ypos(globalStore, function (globalStore, _result45) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    return function () {
                                                        return _k40(globalStore, {
                                                            shape: _result41,
                                                            static: !0,
                                                            dims: [
                                                                _result42,
                                                                _result43
                                                            ],
                                                            x: _result44,
                                                            y: _result45
                                                        });
                                                    };
                                                }, _address167.concat('_174'), ad.scalar.sub(worldHeight, 78.5));
                                            };
                                        }, _address167.concat('_173'), ad.scalar.add(ad.scalar.div(worldWidth, 2), 7));
                                    };
                                }, _address167.concat('_172'));
                            };
                        }, _address167.concat('_171'));
                    };
                }, _address167.concat('_170'));
            };
        };
        var bottom1 = function bottom1(globalStore, _k34, _address168) {
            var _currentAddress = _address168;
            _addr.save(_globalCurrentAddress, _address168);
            return function () {
                return shape(globalStore, function (globalStore, _result35) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return dimB(globalStore, function (globalStore, _result36) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            return function () {
                                return dim_static(globalStore, function (globalStore, _result37) {
                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                    return function () {
                                        return xpos(globalStore, function (globalStore, _result38) {
                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                            return function () {
                                                return ypos(globalStore, function (globalStore, _result39) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    return function () {
                                                        return _k34(globalStore, {
                                                            shape: _result35,
                                                            static: !0,
                                                            dims: [
                                                                ad.scalar.add(_result36, 10),
                                                                _result37
                                                            ],
                                                            x: _result38,
                                                            y: _result39
                                                        });
                                                    };
                                                }, _address168.concat('_179'), ad.scalar.sub(worldHeight, 20));
                                            };
                                        }, _address168.concat('_178'), ad.scalar.div(worldWidth, 2));
                                    };
                                }, _address168.concat('_177'));
                            };
                        }, _address168.concat('_176'));
                    };
                }, _address168.concat('_175'));
            };
        };
        var top1 = function top1(globalStore, _k28, _address169) {
            var _currentAddress = _address169;
            _addr.save(_globalCurrentAddress, _address169);
            return function () {
                return shape(globalStore, function (globalStore, _result29) {
                    _addr.save(_globalCurrentAddress, _currentAddress);
                    return function () {
                        return dimA(globalStore, function (globalStore, _result30) {
                            _addr.save(_globalCurrentAddress, _currentAddress);
                            return function () {
                                return dim_static(globalStore, function (globalStore, _result31) {
                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                    return function () {
                                        return xpos(globalStore, function (globalStore, _result32) {
                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                            return function () {
                                                return ypos(globalStore, function (globalStore, _result33) {
                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                    return function () {
                                                        return _k28(globalStore, {
                                                            shape: _result29,
                                                            static: !0,
                                                            dims: [
                                                                ad.scalar.add(_result30, 10),
                                                                _result31
                                                            ],
                                                            x: _result32,
                                                            y: _result33
                                                        });
                                                    };
                                                }, _address169.concat('_184'), ad.scalar.sub(worldHeight, 80));
                                            };
                                        }, _address169.concat('_183'), ad.scalar.div(worldWidth, 2));
                                    };
                                }, _address169.concat('_182'));
                            };
                        }, _address169.concat('_181'));
                    };
                }, _address169.concat('_180'));
            };
        };
        return function () {
            return top1(globalStore, function (globalStore, _result23) {
                _addr.save(_globalCurrentAddress, _currentAddress);
                return function () {
                    return bottom1(globalStore, function (globalStore, _result24) {
                        _addr.save(_globalCurrentAddress, _currentAddress);
                        return function () {
                            return body(globalStore, function (globalStore, _result25) {
                                _addr.save(_globalCurrentAddress, _currentAddress);
                                return function () {
                                    return eye1(globalStore, function (globalStore, _result26) {
                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                        return function () {
                                            return eye2(globalStore, function (globalStore, _result27) {
                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                var robot1 = [
                                                    ground,
                                                    _result23,
                                                    _result24,
                                                    _result25,
                                                    _result26,
                                                    _result27
                                                ];
                                                var bottom2 = function bottom2(globalStore, _k17, _address170) {
                                                    var _currentAddress = _address170;
                                                    _addr.save(_globalCurrentAddress, _address170);
                                                    return function () {
                                                        return shape(globalStore, function (globalStore, _result18) {
                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                            return function () {
                                                                return dimB(globalStore, function (globalStore, _result19) {
                                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                                    return function () {
                                                                        return dim_static(globalStore, function (globalStore, _result20) {
                                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                                            return function () {
                                                                                return xpos(globalStore, function (globalStore, _result21) {
                                                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                    return function () {
                                                                                        return ypos(globalStore, function (globalStore, _result22) {
                                                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                            return function () {
                                                                                                return _k17(globalStore, {
                                                                                                    shape: _result18,
                                                                                                    static: !0,
                                                                                                    dims: [
                                                                                                        ad.scalar.sub(_result19, 10),
                                                                                                        _result20
                                                                                                    ],
                                                                                                    x: _result21,
                                                                                                    y: _result22
                                                                                                });
                                                                                            };
                                                                                        }, _address170.concat('_194'), ad.scalar.sub(worldHeight, 20));
                                                                                    };
                                                                                }, _address170.concat('_193'), ad.scalar.div(worldWidth, 2));
                                                                            };
                                                                        }, _address170.concat('_192'));
                                                                    };
                                                                }, _address170.concat('_191'));
                                                            };
                                                        }, _address170.concat('_190'));
                                                    };
                                                };
                                                var top2 = function top2(globalStore, _k11, _address171) {
                                                    var _currentAddress = _address171;
                                                    _addr.save(_globalCurrentAddress, _address171);
                                                    return function () {
                                                        return shape(globalStore, function (globalStore, _result12) {
                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                            return function () {
                                                                return dimA(globalStore, function (globalStore, _result13) {
                                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                                    return function () {
                                                                        return dim_static(globalStore, function (globalStore, _result14) {
                                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                                            return function () {
                                                                                return xpos(globalStore, function (globalStore, _result15) {
                                                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                    return function () {
                                                                                        return ypos(globalStore, function (globalStore, _result16) {
                                                                                            _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                            return function () {
                                                                                                return _k11(globalStore, {
                                                                                                    shape: _result12,
                                                                                                    static: !0,
                                                                                                    dims: [
                                                                                                        ad.scalar.sub(_result13, 10),
                                                                                                        _result14
                                                                                                    ],
                                                                                                    x: _result15,
                                                                                                    y: _result16
                                                                                                });
                                                                                            };
                                                                                        }, _address171.concat('_199'), ad.scalar.sub(worldHeight, 80));
                                                                                    };
                                                                                }, _address171.concat('_198'), ad.scalar.div(worldWidth, 2));
                                                                            };
                                                                        }, _address171.concat('_197'));
                                                                    };
                                                                }, _address171.concat('_196'));
                                                            };
                                                        }, _address171.concat('_195'));
                                                    };
                                                };
                                                return function () {
                                                    return top2(globalStore, function (globalStore, _result6) {
                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                        return function () {
                                                            return bottom2(globalStore, function (globalStore, _result7) {
                                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                                return function () {
                                                                    return body(globalStore, function (globalStore, _result8) {
                                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                                        return function () {
                                                                            return eye1(globalStore, function (globalStore, _result9) {
                                                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                return function () {
                                                                                    return eye2(globalStore, function (globalStore, _result10) {
                                                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                        var robot2 = [
                                                                                            ground,
                                                                                            _result6,
                                                                                            _result7,
                                                                                            _result8,
                                                                                            _result9,
                                                                                            _result10
                                                                                        ];
                                                                                        var whichRobot = function whichRobot(globalStore, _k4, _address172) {
                                                                                            var _currentAddress = _address172;
                                                                                            _addr.save(_globalCurrentAddress, _address172);
                                                                                            return function () {
                                                                                                return flip(globalStore, function (globalStore, _result5) {
                                                                                                    _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                                    return function () {
                                                                                                        return _result5 ? _k4(globalStore, robot1) : _k4(globalStore, robot2);
                                                                                                    };
                                                                                                }, _address172.concat('_205'));
                                                                                            };
                                                                                        };
                                                                                        return function () {
                                                                                            return whichRobot(globalStore, function (globalStore, fallingWorld) {
                                                                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                                return function () {
                                                                                                    return print(globalStore, function (globalStore, _dummy3) {
                                                                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                                        return function () {
                                                                                                            return print(globalStore, function (globalStore, _dummy2) {
                                                                                                                _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                                                return function () {
                                                                                                                    return print(globalStore, function (globalStore, _dummy1) {
                                                                                                                        _addr.save(_globalCurrentAddress, _currentAddress);
                                                                                                                        return function () {
                                                                                                                            return _k0(globalStore, physics.animate(10, fallingWorld));
                                                                                                                        };
                                                                                                                    }, _address0.concat('_209'), ad.scalar.sub(fallingWorld[1].dims[0], fallingWorld[2].dims[0]));
                                                                                                                };
                                                                                                            }, _address0.concat('_208'), fallingWorld[2].dims[0]);
                                                                                                        };
                                                                                                    }, _address0.concat('_207'), fallingWorld[1].dims[0]);
                                                                                                };
                                                                                            }, _address0.concat('_206'));
                                                                                        };
                                                                                    }, _address0.concat('_204'));
                                                                                };
                                                                            }, _address0.concat('_203'));
                                                                        };
                                                                    }, _address0.concat('_202'));
                                                                };
                                                            }, _address0.concat('_201'));
                                                        };
                                                    }, _address0.concat('_200'));
                                                };
                                            }, _address0.concat('_189'));
                                        };
                                    }, _address0.concat('_188'));
                                };
                            }, _address0.concat('_187'));
                        };
                    }, _address0.concat('_186'));
                };
            }, _address0.concat('_185'));
        };
    });
});

main({})(__runner__)({}, topK, '');